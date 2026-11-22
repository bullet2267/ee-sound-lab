# -*- coding: utf-8 -*-
# Снятие аудио с платы THOR: WS /ctrl (mg+зажигание) -> HTTP /drag -> /dump.wav.
# Raw-websocket на сокетах (без зависимостей). Запуск: python -I board_cap.py OUT.wav [sec]
import socket, base64, os, struct, time, urllib.request, sys
HOST='10.110.12.106'; PORT=80
def ws_connect(path='/ctrl'):
    s=socket.create_connection((HOST,PORT),timeout=8)
    key=base64.b64encode(os.urandom(16)).decode()
    req=('GET %s HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
         'Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n'%(path,HOST,key))
    s.sendall(req.encode())
    resp=b''
    while b'\r\n\r\n' not in resp: resp+=s.recv(256)
    assert b'101' in resp.split(b'\r\n')[0], resp[:80]
    return s
def ws_send(s,payload):          # masked binary frame
    mask=os.urandom(4); n=len(payload)
    hdr=bytes([0x82, 0x80|n])    # FIN+binary, mask bit + len (n<126)
    masked=bytes(payload[i]^mask[i%4] for i in range(n))
    s.sendall(hdr+mask+masked)
def ctrl(frames):
    s=ws_connect('/ctrl')
    for f in frames:
        ws_send(s,bytes(f)); time.sleep(0.15)
    time.sleep(0.3); s.close()
def http(path):
    return urllib.request.urlopen('http://%s%s'%(HOST,path),timeout=25).read()
if __name__=='__main__':
    out=sys.argv[1] if len(sys.argv)>1 else 'renders/BOARD_new.wav'
    sec=int(sys.argv[2]) if len(sys.argv)>2 else 20
    mg=int(sys.argv[3],0) if len(sys.argv)>3 else 0xCC
    print('1) зажигание on, ПОТОМ mg=0x%02x (%.2f) — иначе старт затирает mg=0.50'%(mg,mg/255))
    ctrl([[0x06,0x01,0x00]]); time.sleep(0.4); ctrl([[0x01,mg]])
    time.sleep(0.5)
    print('2) /drag?sec=%d'%sec); print('   ',http('/drag?sec=%d'%sec).decode('ascii','ignore'))
    print('3) ждём %dс заезд...'%(sec+1)); time.sleep(sec+1.5)
    print('4) /dump.wav'); wav=http('/dump.wav')
    os.makedirs('renders',exist_ok=True)
    open(out,'wb').write(wav)
    print('   сохранил %s (%d байт)'%(out,len(wav)))
