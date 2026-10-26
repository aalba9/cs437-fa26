"""Picamera2 + OpenCV + signed-int8 SSD MobileNet v2 with LiteRT CPU kernels.
One full-frame inference per update; FPS includes encoding and rate limiting.
"""
import threading
import time
from pathlib import Path
import cv2
import numpy as np
from ai_edge_litert.interpreter import Interpreter

ROOT=Path(__file__).resolve().parents[2]


class Detector:
    def __init__(self,threads=2):
        self.model=Interpreter(model_path=str(ROOT/'models/detect.tflite'),num_threads=threads)
        self.model.allocate_tensors()
        self.input=self.model.get_input_details()[0]
        outputs=self.model.get_output_details()
        # Pinned TF2 export lists scores, boxes, count, classes (not SSD order).
        self.outputs=[outputs[i] for i in (1,3,0)]
        self.labels=(ROOT/'models/labels.txt').read_text().splitlines()
        self.height,self.width=self.input['shape'][1:3]

    # 0.55 exceeds the 0.50 "even odds" reference to filter weak person hits.
    # Confidence is not a calibrated probability; this cutoff applies to all classes.
    def detect(self,bgr,threshold=.55):
        rgb=cv2.cvtColor(cv2.resize(bgr,(self.width,self.height)),cv2.COLOR_BGR2RGB)
        if self.input['dtype']==np.float32: rgb=(rgb.astype(np.float32)-127.5)/127.5
        self.model.set_tensor(self.input['index'],rgb[None,...])
        begin=time.monotonic(); self.model.invoke(); elapsed=time.monotonic()-begin
        boxes=self.model.get_tensor(self.outputs[0]['index'])[0]
        classes=self.model.get_tensor(self.outputs[1]['index'])[0]
        scores=self.model.get_tensor(self.outputs[2]['index'])[0]
        found=[]
        for box,cls,score in zip(boxes,classes,scores):
            if score>=threshold:
                c=int(cls)
                found.append({'label':self.labels[c] if c<len(self.labels) else str(c),
                              'score':round(float(score),3),'box':[float(v) for v in box]})
        return found,elapsed


class Vision:
    def __init__(self,robot):
        self.robot=robot; self.lock=threading.Lock(); self.quit=threading.Event()
        self.latest={'detections':[],'fps':0.,'inference_ms':0.,'at':0.,'front_at':0.,'stop_sign':None,'error':'Starting camera'}
        self.jpeg=None;self.raw_jpeg=None
        self.thread=threading.Thread(target=self._run,daemon=True); self.thread.start()

    def status(self):
        with self.lock: return dict(self.latest)

    def _run(self):
        camera=None
        try:
            from picamera2 import Picamera2
            cv2.setNumThreads(1)
            detector=Detector(threads=self.robot.cfg.get('vision_threads',2))
            period=1/self.robot.cfg.get('vision_fps',8.)
            camera=Picamera2()
            # Picamera2 RGB888 produces byte-ordered BGR, matching OpenCV.
            camera.configure(camera.create_video_configuration(main={'size':(640,480),'format':'RGB888'},buffer_count=2))
            camera.start(); time.sleep(.5)
            last_stop_sign=None; previous_at=None; fps=0.
            while not self.quit.is_set():
                begin=time.monotonic()
                pan=self.robot.pan
                centered=self.robot.pan_centered_at
                frame=camera.capture_array('main')
                found,elapsed=detector.detect(frame,self.robot.cfg['detection_threshold'])
                raw_ok,raw_jpg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,95])
                signs=[obj for obj in found if obj['label']=='stop sign']
                if signs:
                    last_stop_sign={'at':begin,'pan_deg':pan,'detections':signs}
                h,w=frame.shape[:2]
                for obj in found:
                    y1,x1,y2,x2=obj['box']
                    a=(int(x1*w),int(y1*h)); b=(int(x2*w),int(y2*h))
                    color=(0,80,255) if obj['label'] in ('person','stop sign') else (30,220,80)
                    cv2.rectangle(frame,a,b,color,2)
                    cv2.putText(frame,f"{obj['label']} {obj['score']:.2f}",(a[0],max(18,a[1]-5)),cv2.FONT_HERSHEY_SIMPLEX,.5,color,2)
                cv2.putText(frame,f'{fps:.1f} FPS',(12,26),cv2.FONT_HERSHEY_SIMPLEX,.7,(30,220,80),2)
                ok,jpg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,80])
                published_at=time.monotonic()
                fps=1/(published_at-previous_at) if previous_at is not None else 0.
                previous_at=published_at
                front=pan==0 and self.robot.pan==0 and centered==self.robot.pan_centered_at and begin-centered>.22
                with self.lock:
                    self.latest={'detections':found,'fps':round(fps,2),'inference_ms':round(elapsed*1000,2),
                                 'at':published_at,'front_at':begin if front else 0.,'stop_sign':last_stop_sign,'error':None}
                    if ok: self.jpeg=jpg.tobytes()
                    if raw_ok:self.raw_jpeg=raw_jpg.tobytes()
                self.quit.wait(max(0.,period-(time.monotonic()-begin)))
        except Exception as exc:
            with self.lock: self.latest['error']=str(exc)
        finally:
            if camera:
                camera.stop(); camera.close()

    def close(self):
        self.quit.set(); self.thread.join(timeout=4)
