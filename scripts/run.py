"""Run isolated Gazebo/ROS processes; optionally record the actual Gazebo GUI."""
import argparse
import glob
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import uuid

ROOT=Path('/workspace/range')
OUT=Path('/workspace/output')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--gui',action='store_true')
    parser.add_argument('--maximum-wall-seconds',type=float,default=190.)
    args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    if any(p.name!='console.log' for p in OUT.iterdir()):
        raise SystemExit('Previous results preserved. Select a NEW output directory.')
    with (OUT/'run-start.json').open('x') as stream:
        json.dump({'gui_requested':args.gui,'wall_started':time.time()},stream)
    env=os.environ.copy()
    env.update(PYTHONPATH=str(ROOT/'src')+os.pathsep+env.get('PYTHONPATH',''),PYTHONDONTWRITEBYTECODE='1',RANGE_ROOT=str(ROOT),RANGE_OUTPUT=str(OUT),
               LIBGL_ALWAYS_SOFTWARE='1',GALLIUM_DRIVER='llvmpipe',ROS_DOMAIN_ID='78',
               ROS_LOCALHOST_ONLY='1',GZ_PARTITION='range_guide_'+uuid.uuid4().hex,QT_QPA_PLATFORM='xcb' if args.gui else 'offscreen')
    processes=[];logs=[];video=None;began=time.monotonic();completed=False

    def start(name,command):
        stream=(OUT/(name+'.log')).open('w');logs.append(stream)
        process=subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,process));return process

    try:
        if args.gui:
            env['DISPLAY']=':99'
            start('xvfb',['Xvfb',':99','-screen','0','1280x720x24','-nolisten','tcp','-ac','+extension','GLX','-noreset'])
            for _ in range(30):
                probe=subprocess.run(['xdotool','getdisplaygeometry'],env=env,capture_output=True)
                if probe.returncode==0:break
                time.sleep(.2)
            else:raise RuntimeError('Virtual display unavailable; see xvfb.log')
        start('gazebo',['gz','sim','-s','-r','--headless-rendering','-v','2',str(ROOT/'worlds/ward-lidar.sdf')])
        start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:='+str(ROOT/'config/bridge.yaml')])
        if args.gui:
            source=Path(glob.glob('/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim*/gui/gui.config')[0]).read_text()
            source=source.replace('show_again="true"','show_again="false"').replace('<width>1000</width>','<width>1280</width>').replace('<height>845</height>','<height>720</height>')
            source=re.sub(r'<camera_pose>.*?</camera_pose>','<camera_pose>4.5 -10 10 0 0.78 1.5707963267948966</camera_pose>',source)
            source=re.sub(r'<plugin filename="ComponentInspector".*?</plugin>','',source,flags=re.S)
            config=OUT/'gui.config';config.write_text(source)
            start('gui',['gz','sim','-g','--gui-config',str(config)])
            time.sleep(8.)
            subprocess.run(['ffmpeg','-y','-loglevel','error','-f','x11grab','-video_size','1280x720','-i',':99.0','-frames:v','1',str(OUT/'gui-ready.png')],env=env,check=True)
            video=start('screen-recording',['ffmpeg','-y','-loglevel','warning','-f','x11grab','-framerate','20',
                '-video_size','1280x720','-i',':99.0','-an','-c:v','libx264','-preset','ultrafast','-crf','21',
                '-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/'range-guidance-raw-gui.mp4')])
        for role in ('audit','object','guide'):
            start(role,[sys.executable,'-m','ward_range.runtime',role])
        last_report=0.
        while time.monotonic()-began<args.maximum_wall_seconds:
            if (OUT/'result.json').exists():
                result=json.loads((OUT/'result.json').read_text());completed=True
                print(json.dumps({'result':result},ensure_ascii=False),flush=True)
                time.sleep(3.);result=json.loads((OUT/'result.json').read_text());break
            for name,process in processes:
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited with {process.returncode}; inspect {name}.log')
            if time.monotonic()-last_report>10:
                last_report=time.monotonic();trace=OUT/'trace.jsonl'
                last={}
                if trace.exists():
                    lines=trace.read_text().splitlines()
                    if lines:
                        try:last=json.loads(lines[-1])
                        except ValueError:pass
                print(json.dumps({'wall_seconds':round(time.monotonic()-began),'state':last.get('state'),
                                  'sim_elapsed':last.get('elapsed'),'scans':last.get('scans'),'detections':last.get('detections')}),flush=True)
            time.sleep(.25)
        if not completed:raise RuntimeError('Bounded wall-time limit reached; no successful result claimed')
    finally:
        # Flush MP4 before closing the simulation; only terminate processes started here.
        ordered=([('screen-recording',video)] if video else [])+[(n,p) for n,p in reversed(processes) if p is not video]
        for name,process in ordered:
            if process is not None and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=8 if process is video else 3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait()
        for stream in logs:stream.close()
        video_path=OUT/'range-guidance-raw-gui.mp4'
        probe=None
        if video_path.exists() and video_path.stat().st_size:
            check=subprocess.run(['ffprobe','-v','error','-show_entries','format=duration:stream=width,height,r_frame_rate','-of','json',str(video_path)],capture_output=True,text=True)
            if check.returncode==0:probe=json.loads(check.stdout)
        duration=float(probe['format']['duration']) if probe else None
        (OUT/'runner.json').write_text(json.dumps({'completed':completed,'gui_requested':args.gui,'video_probe':probe,
            'video_file_created':video_path.exists(),'duration_requirement_met':duration is not None and 60<=duration<=180,
            'wall_seconds':round(time.monotonic()-began,3),'video_clock':'wall time, no speed changes, no rendered overlays',
            'processes':{n:p.returncode for n,p in processes}},indent=2))
    raise SystemExit(0 if result['success'] else 1)

if __name__=='__main__':main()
