"""Fetch the exact pretrained Google Coral test model used by this lab."""
import hashlib,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
BASE='https://raw.githubusercontent.com/google-coral/test_data/104342d2d3480b3e66203073dac24f4e2dbb4c41/'
SOURCES={'models/detect.tflite':BASE+'tf2_ssd_mobilenet_v2_coco17_ptq.tflite','models/labels.txt':BASE+'coco_labels.txt'}
def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()
def main():
    for line in (ROOT/'model-sha256.txt').read_text().splitlines():
        expected,name=line.split();target=ROOT/name
        if target.exists() and sha(target)==expected:
            print('Verified:',name);continue
        target.parent.mkdir(parents=True,exist_ok=True)
        temporary=target.with_suffix(target.suffix+'.download')
        try:
            with urllib.request.urlopen(SOURCES[name],timeout=30) as response,temporary.open('wb') as stream:
                while chunk:=response.read(1024*1024):stream.write(chunk)
            if sha(temporary)!=expected:raise RuntimeError('Checksum mismatch: '+name)
            temporary.replace(target);print('Downloaded and verified:',name)
        finally:
            if temporary.exists():temporary.unlink()
if __name__=='__main__':main()
