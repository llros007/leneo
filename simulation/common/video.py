"""Stream timestamped RGB simulation frames directly into a portable H.264 MP4."""
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image,ImageDraw,ImageFont


class VideoWriter:
    def __init__(self,path,width,height,fps,engine):
        self.path=Path(path).resolve();self.path.parent.mkdir(parents=True,exist_ok=True)
        self.width=width;self.height=height;self.fps=fps;self.engine=engine;self.frames=0;self.closed=False
        self.font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
        self.log=self.path.with_suffix('.encode.log').open('w')
        self.process=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24','-video_size',f'{width}x{height}','-framerate',str(fps),'-i','pipe:0','-an','-c:v','libx264','-preset','fast','-crf','18','-threads','4','-pix_fmt','yuv420p','-movflags','+faststart',str(self.path)],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=self.log)

    @property
    def next_time(self):return self.frames/self.fps

    def write(self,frame,simtime,phase):
        frame=np.asarray(frame,dtype=np.uint8)
        if frame.shape[:2]!=(self.height,self.width):raise ValueError(f'Unexpected video size {frame.shape}')
        picture=Image.fromarray(frame[:,:,:3]).convert('RGB');draw=ImageDraw.Draw(picture)
        title=f'{self.engine}  |  Pick orange from floor to bar  |  {simtime:05.1f}s  |  {phase.replace("-"," ")}'
        box=draw.textbbox((0,0),title,font=self.font)
        draw.rectangle((10,10,min(self.width-10,box[2]+30),45),fill=(22,26,30))
        draw.text((20,17),title,font=self.font,fill=(248,248,248))
        self.process.stdin.write(picture.tobytes());self.frames+=1

    def close(self):
        if self.closed:return
        self.closed=True;self.process.stdin.close();code=self.process.wait(timeout=30);self.log.close()
        if code:raise RuntimeError(f'ffmpeg failed ({code}), see {self.path.with_suffix(".encode.log")}')

    def report(self):return dict(path=str(self.path),frames=self.frames,fps=self.fps,duration=self.frames/self.fps,width=self.width,height=self.height)
