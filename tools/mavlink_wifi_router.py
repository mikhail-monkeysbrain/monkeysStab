#!/usr/bin/env python3
"""
monkeysStab MAVLink byte router.

Один процесс владеет физическим UART FC и прозрачно раздаёт байты:
  FC /dev/ttyAMA0 <-> локальный TCP 127.0.0.1:5760
                  -> UDP Mission Planner :14550

TCP-клиент monkeysStab получает полный поток FC и может отправлять MAVLink обратно.
UDP GCS получает телеметрию; любой пакет, пришедший на UDP 14550, передаётся FC.
"""
import argparse, os, select, socket, termios, time, subprocess, errno, struct
from collections import Counter

BAUD={460800:termios.B460800,115200:termios.B115200,57600:termios.B57600}

# Passive MAVLink v1/v2 framing counter.  It never writes to FC and is used
# only to prove what message rates already exist at the physical UART.
class MavCommandSpy:
    """Диагностика команд, которыми TCP/UDP-клиенты меняют MAVLink stream rates."""
    def __init__(self, label):
        self.label=label
        self.buf=bytearray()

    def feed(self, data):
        self.buf.extend(data)
        while self.buf:
            try:
                i=next(i for i,b in enumerate(self.buf) if b in (0xFE,0xFD))
            except StopIteration:
                self.buf.clear(); return
            if i: del self.buf[:i]
            if len(self.buf)<2: return
            magic=self.buf[0]; plen=self.buf[1]
            if magic==0xFE:
                total=plen+8
                if len(self.buf)<total: return
                msgid=self.buf[5]; sysid=self.buf[3]; compid=self.buf[4]
                payload=bytes(self.buf[6:6+plen])
            else:
                if len(self.buf)<10: return
                incompat=self.buf[2]
                total=plen+12+(13 if (incompat & 0x01) else 0)
                if len(self.buf)<total: return
                msgid=self.buf[7] | (self.buf[8]<<8) | (self.buf[9]<<16)
                sysid=self.buf[5]; compid=self.buf[6]
                payload=bytes(self.buf[10:10+plen])
            if msgid==76 and len(payload)>=33:  # COMMAND_LONG
                command=struct.unpack_from("<H",payload,28)[0]
                if command==511:  # MAV_CMD_SET_MESSAGE_INTERVAL
                    message_id=struct.unpack_from("<f",payload,0)[0]
                    interval_us=struct.unpack_from("<f",payload,4)[0]
                    print(f"FC_TX_RATE_CMD source={self.label} sys={sysid} comp={compid} "
                          f"SET_MESSAGE_INTERVAL msgid={message_id:.0f} interval_us={interval_us:.0f}",
                          flush=True)
            elif msgid==66 and len(payload)>=6:  # REQUEST_DATA_STREAM
                rate=struct.unpack_from("<H",payload,0)[0]
                stream_id=payload[4]; start_stop=payload[5]
                print(f"FC_TX_RATE_CMD source={self.label} sys={sysid} comp={compid} "
                      f"REQUEST_DATA_STREAM stream={stream_id} rate_hz={rate} start={start_stop}",
                      flush=True)
            del self.buf[:total]


class MavRxCounter:
    def __init__(self):
        self.buf=bytearray()
        self.counts=Counter()
        self.bad_prefix=0

    def feed(self,data):
        self.buf.extend(data)
        while self.buf:
            try:
                i=next(i for i,b in enumerate(self.buf) if b in (0xFE,0xFD))
            except StopIteration:
                self.bad_prefix+=len(self.buf); self.buf.clear(); return
            if i:
                self.bad_prefix+=i; del self.buf[:i]
            if len(self.buf)<2: return
            magic=self.buf[0]; payload=self.buf[1]
            if magic==0xFE:
                total=payload+8
                if len(self.buf)<total: return
                msgid=self.buf[5]
            else:
                if len(self.buf)<10: return
                incompat=self.buf[2]
                total=payload+12+(13 if (incompat & 0x01) else 0)
                if len(self.buf)<total: return
                msgid=self.buf[7] | (self.buf[8]<<8) | (self.buf[9]<<16)
            self.counts[msgid]+=1
            del self.buf[:total]


def serial_open(path,baud):
    fd=os.open(path,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
    a=termios.tcgetattr(fd)
    a[0]=0; a[1]=0; a[3]=0
    a[2]=termios.CLOCAL|termios.CREAD|termios.CS8
    a[4]=BAUD[baud]; a[5]=BAUD[baud]
    a[6][termios.VMIN]=0; a[6][termios.VTIME]=0
    termios.tcsetattr(fd,termios.TCSANOW,a)
    termios.tcflush(fd,termios.TCIFLUSH)
    return fd

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--serial",default="/dev/ttyAMA0")
    p.add_argument("--baud",type=int,default=460800)
    p.add_argument("--tcp-port",type=int,default=5760)
    p.add_argument("--udp-port",type=int,default=14550)
    p.add_argument("--gcs-ip",default="",help="необязательно: IP ПК для немедленной отправки телеметрии")
    a=p.parse_args()
    if a.baud not in BAUD: raise SystemExit(f"baud {a.baud} не поддержан")

    ser=serial_open(a.serial,a.baud)
    srv=socket.socket(); srv.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    srv.bind(("127.0.0.1",a.tcp_port)); srv.listen(4); srv.setblocking(False)
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    udp.bind(("0.0.0.0",a.udp_port)); udp.setblocking(False)

    clients=[]; client_tx={}; client_spy={}; gcs=set(); udp_spy={}
    rx_counter=MavRxCounter()
    rx_stat_t=time.monotonic()
    rx_stat_counts=Counter()
    if a.gcs_ip: gcs.add((a.gcs_ip,a.udp_port))
    def local_ipv4_addresses():
        addrs=[]
        try:
            out=subprocess.check_output(["hostname","-I"],text=True,timeout=2)
            for token in out.split():
                if token.count(".")==3 and token!="127.0.0.1" and token not in addrs:
                    addrs.append(token)
        except Exception:
            pass
        return addrs

    ips=local_ipv4_addresses()

    print("======================================================================")
    print("monkeysStab — MAVLink Wi-Fi router")
    print("======================================================================")
    print(f"FC UART:              {a.serial} @ {a.baud}")
    print(f"monkeysStab local TCP: 127.0.0.1:{a.tcp_port}")
    print(f"Mission Planner UDP:   port {a.udp_port}")
    print()
    if ips:
        print("ПОДКЛЮЧЕНИЕ MISSION PLANNER:")
        for ip in ips:
            print(f"  UDPCl -> {ip}:{a.udp_port}")
    else:
        print("ПОДКЛЮЧЕНИЕ MISSION PLANNER:")
        print(f"  IP RPi не определён автоматически; порт UDP {a.udp_port}")
    if a.gcs_ip:
        print()
        print(f"Предустановленный адрес ПК: {a.gcs_ip}:{a.udp_port}")
    print()
    print("В Mission Planner выберите UDPCl, укажите один из IP выше и порт "
          f"{a.udp_port}.")
    print("======================================================================")
    print("Ожидание подключений...",flush=True)

    try:
        while True:
            r=[ser,srv,udp]+clients
            ready,_,_=select.select(r,[],[],0.5)
            for x in ready:
                if x==ser:
                    try: data=os.read(ser,65536)
                    except BlockingIOError: data=b""
                    if not data: continue
                    rx_counter.feed(data)
                    now=time.monotonic()
                    if now-rx_stat_t >= 5.0:
                        dt=now-rx_stat_t
                        ids=((30,"ATTITUDE"),(105,"HIGHRES_IMU"),
                             (32,"LOCAL_POSITION_NED"),(193,"EKF_STATUS_REPORT"))
                        parts=[]
                        for mid,name in ids:
                            n=rx_counter.counts[mid]-rx_stat_counts[mid]
                            parts.append(f"{name}={n/dt:.1f}Hz({n})")
                        print("FC_RX_RATE " + " ".join(parts) +
                              f" clients={len(clients)}",flush=True)
                        rx_stat_counts=rx_counter.counts.copy()
                        rx_stat_t=now
                    dead=[]
                    for c in clients:
                        try:
                            # Clients are non-blocking. sendall() is the wrong
                            # primitive here: a temporary EAGAIN used to eject
                            # a healthy client from the fan-out.  A MAVLink
                            # consumer must either receive the complete serial
                            # chunk or be disconnected explicitly.
                            n=c.send(data)
                            if n != len(data):
                                raise OSError(errno.ENOBUFS,
                                              f"short nonblocking send {n}/{len(data)}")
                            st=client_tx.setdefault(c,{"bytes":0,"drops":0})
                            st["bytes"]+=n
                        except (BlockingIOError,InterruptedError):
                            st=client_tx.setdefault(c,{"bytes":0,"drops":0})
                            st["drops"]+=1
                            # Do not silently remove a client on transient
                            # backpressure.  Its TCP receive buffer can recover.
                            continue
                        except OSError:
                            dead.append(c)
                    for c in dead:
                        try:c.close()
                        except:pass
                        client_tx.pop(c,None)
                        client_spy.pop(c,None)
                        if c in clients: clients.remove(c)
                    for peer in list(gcs):
                        try: udp.sendto(data,peer)
                        except OSError: pass
                elif x==srv:
                    c,addr=srv.accept()
                    c.setblocking(False)
                    # Give short FC bursts room even if a consumer is briefly
                    # busy; this is still bounded kernel buffering.
                    c.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,262144)
                    clients.append(c)
                    client_tx[c]={"bytes":0,"drops":0}
                    client_spy[c]=MavCommandSpy(f"tcp:{addr[0]}:{addr[1]}")
                    print(f"local client connected: {addr}",flush=True)
                elif x==udp:
                    try: data,peer=udp.recvfrom(65536)
                    except BlockingIOError: continue
                    gcs.add(peer)
                    if data:
                        spy=udp_spy.setdefault(peer,MavCommandSpy(f"udp:{peer[0]}:{peer[1]}"))
                        spy.feed(data)
                        try: os.write(ser,data)
                        except BlockingIOError: pass
                else:
                    c=x
                    try: data=c.recv(65536)
                    except BlockingIOError: continue
                    except OSError: data=b""
                    if not data:
                        try:c.close()
                        except:pass
                        client_tx.pop(c,None)
                        if c in clients: clients.remove(c)
                        continue
                    spy=client_spy.get(c)
                    if spy is not None: spy.feed(data)
                    try: os.write(ser,data)
                    except BlockingIOError: pass
    finally:
        for c in clients: c.close()
        srv.close(); udp.close(); os.close(ser)

if __name__=="__main__": main()
