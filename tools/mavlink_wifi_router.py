#!/usr/bin/env python3
"""
monkeysStab MAVLink byte router.

Один процесс владеет физическим UART FC и прозрачно раздаёт байты:
  FC /dev/ttyAMA0 <-> локальный TCP 127.0.0.1:5760
                  -> UDP Mission Planner :14550

TCP-клиент monkeysStab получает полный поток FC и может отправлять MAVLink обратно.
UDP GCS получает телеметрию; любой пакет, пришедший на UDP 14550, передаётся FC.
"""
import argparse, os, select, socket, termios, time, subprocess

BAUD={460800:termios.B460800,115200:termios.B115200,57600:termios.B57600}

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

    clients=[]; gcs=set()
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
                    dead=[]
                    for c in clients:
                        try: c.sendall(data)
                        except OSError: dead.append(c)
                    for c in dead:
                        try:c.close()
                        except:pass
                        if c in clients: clients.remove(c)
                    for peer in list(gcs):
                        try: udp.sendto(data,peer)
                        except OSError: pass
                elif x==srv:
                    c,addr=srv.accept(); c.setblocking(False); clients.append(c)
                    print(f"local client connected: {addr}",flush=True)
                elif x==udp:
                    try: data,peer=udp.recvfrom(65536)
                    except BlockingIOError: continue
                    gcs.add(peer)
                    if data:
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
                        if c in clients: clients.remove(c)
                        continue
                    try: os.write(ser,data)
                    except BlockingIOError: pass
    finally:
        for c in clients: c.close()
        srv.close(); udp.close(); os.close(ser)

if __name__=="__main__": main()
