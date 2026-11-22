# -*- coding: utf-8 -*-
# Аудит аудио: метрики дефектов + спектрограмма PNG (которую Claude читает глазами).
# Запуск: python -I audio_audit.py FILE.wav [REF.wav]
import sys, os, wave, struct, zlib, math
import numpy as np
SR=22050
def load(p):
    w=wave.open(p,'rb'); sr=w.getframerate()
    x=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(np.float64); w.close()
    return sr,x
def band(s,sr,lo,hi):
    F=np.fft.rfft(s); fr=np.fft.rfftfreq(len(s),1.0/sr)
    F[(fr<lo)|(fr>=hi)]=0
    return np.sqrt((np.fft.irfft(F,len(s))**2).mean())
# ---------- PNG (RGB, без зависимостей) ----------
def write_png(path, rgb):
    H,W,_=rgb.shape; raw=bytearray()
    rb=rgb.astype(np.uint8)
    for y in range(H):
        raw.append(0); raw.extend(rb[y].tobytes())
    comp=zlib.compress(bytes(raw),6)
    def ch(t,d): return struct.pack('>I',len(d))+t+d+struct.pack('>I',zlib.crc32(t+d)&0xffffffff)
    ihdr=struct.pack('>IIBBBBB',W,H,8,2,0,0,0)
    open(path,'wb').write(b'\x89PNG\r\n\x1a\n'+ch(b'IHDR',ihdr)+ch(b'IDAT',comp)+ch(b'IEND',b''))
def magma(v):  # v in 0..1 -> RGB, высокий контраст
    v=np.clip(v,0,1)
    r=np.clip(1.5*v-0.2,0,1); g=np.clip(1.6*v-0.6,0,1); b=np.clip(0.9*np.sin(3.14*v*0.9),0,1)
    return (np.stack([r,g,b],-1)*255)
def spectrogram(x,sr,path,fmax=8000,W=1100,H=440,title=''):
    win=1024; hop=256; w=np.hanning(win)
    n=1+(len(x)-win)//hop
    frames=np.empty((n,win))
    for i in range(n): frames[i]=x[i*hop:i*hop+win]*w
    S=np.abs(np.fft.rfft(frames,axis=1))  # n x 513
    fr=np.fft.rfftfreq(win,1.0/sr)
    kmax=np.searchsorted(fr,fmax)
    S=S[:,:kmax]
    S=20*np.log10(S+1e-3)
    # нормализация к 0..1 по перцентилям
    lo,hi=np.percentile(S,5),np.percentile(S,99.5)
    Sn=(S-lo)/(hi-lo+1e-9)
    # ресайз по времени к W, по частоте к H (freq снизу)
    ti=np.linspace(0,n-1,W).astype(int)
    fi=np.linspace(0,kmax-1,H).astype(int)
    img=Sn[ti][:,fi].T[::-1]      # H x W, низ=низкие частоты
    rgb=magma(img)
    write_png(path,rgb)
    return fr[fi][::-1]  # частоты строк (сверху вниз)
def nacf(x,lo_s,hi_s,sr):
    x=x-x.mean(); n=len(x); N=1<<int(np.ceil(np.log2(2*n)))
    F=np.fft.rfft(x,N); ac=np.fft.irfft(F*np.conj(F),N)[:n]; ac=ac/(ac[0]+1e-12)
    lo=int(lo_s*sr); hi=min(int(hi_s*sr),len(ac)-1); seg=ac[lo:hi]
    k=int(np.argmax(seg))+lo; return k/sr,float(ac[k])
def section(x,sr,a,b): return x[int(a*sr):int(b*sr)]
def analyze(path, refpath=None):
    sr,x=load(path); dur=len(x)/sr
    base=os.path.splitext(os.path.basename(path))[0]
    print('='*68); print('%s  %.1fс %dHz  pk %d  DC %.0f'%(base,dur,sr,np.abs(x).max(),x.mean()))
    clip=int((np.abs(x)>=32000).sum())
    print('  клиппинг сэмплов >=32000: %d'%clip)
    # профиль RMS по 0.5с
    h=int(0.5*sr); prof=[int(np.sqrt((x[i*h:(i+1)*h]**2).mean())) for i in range(int(dur*2))]
    print('  RMS по 0.5с:', ' '.join(str(v) for v in prof))
    # тремоло: модуляция огибающей 4-45 Гц (это «хрип/вибрация»)
    e=np.abs(x); k=int(0.0007*sr)|1
    e=np.convolve(e,np.ones(k)/k,'same'); 
    for a,b,nm in [(0.3,2.0,'ХХ'),(6.0,8.0,'средн.газ'),(11.0,13.0,'верх.газ')]:
        s=section(x,sr,a,b)
        if len(s)<4096: continue
        es=np.abs(s); es=np.convolve(es,np.ones(k)/k,'same'); es=(es-es.mean())*np.hanning(len(es))
        F=np.abs(np.fft.rfft(es))**2; frq=np.fft.rfftfreq(len(es),1.0/sr)
        tot=F[(frq>=2)&(frq<=200)].sum()+1e-9
        trem=F[(frq>=4)&(frq<=45)].sum()/tot*100     # доля в тремоло-полосе
        # разброс гармоник при известной f0 (по средним rpm куска нет, ищем f0 в огибающей)
        band_e=F[(frq>=20)&(frq<=400)]; frb=frq[(frq>=20)&(frq<=400)]
        f0=frb[int(np.argmax(band_e))] if len(band_e) else 0
        print('  [%s] тремоло 4-45Гц: %.1f%%  (хаос огибающей; норма эталона <~20%%)  f0огиб=%.1fГц'%(nm,trem,f0))
    # повтор/луп
    lag,r=nacf(section(x,sr,0.3,4.5),0.3,3.0,sr)
    print('  луп-повтор (огиб.): %.3fс r=%.3f  (r>0.4 = слышно зацикливание)'%(lag,r))
    # октавный баланс (ХХ и газ)
    OCT=[(20,50),(50,100),(100,200),(200,400),(400,800),(800,1600),(1600,3200),(3200,6400)]
    for a,b,nm in [(0.3,2.0,'ХХ'),(10.5,12.5,'газ')]:
        s=section(x,sr,a,b); ref=band(s,sr,200,400)
        row=' '.join('%+5.0f'%(20*math.log10(max(band(s,sr,lo,hi),1e-9)/max(ref,1e-9))) for lo,hi in OCT)
        print('  октавы[%s] отн200-400: %s'%(nm,row))
    # спектрограммы
    os.makedirs('audit',exist_ok=True)
    p1='audit/%s_spec.png'%base; spectrogram(x,sr,p1,fmax=8000,title=base)
    p2='audit/%s_idle.png'%base; spectrogram(section(x,sr,0.0,3.0),sr,p2,fmax=4000)
    print('  PNG:', p1, p2)
    if refpath and os.path.exists(refpath):
        p3='audit/REF_spec.png'; sr2,xr=load(refpath); spectrogram(xr,sr2,p3,fmax=8000)
        print('  REF PNG:', p3)
if __name__=='__main__':
    analyze(sys.argv[1], sys.argv[2] if len(sys.argv)>2 else None)
