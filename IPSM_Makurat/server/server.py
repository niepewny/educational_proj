from flask import Flask, request, Response, send_file
from PIL import Image
import threading
import processor
import os


app = Flask(__name__)
num_saved = 0

@app.route("/send", methods=["POST"])
def send():
    img = Image.open(request.files['file'])
    img = img.convert('RGB')
    global num_saved
    num_saved += 1
    path = processor.lr_path + "/" +str(num_saved) + ".jpg"
    img.save(path)
    return Response(status=200)

@app.route("/process")
def startprocess():  
    t = threading.Thread(target=process, daemon=True)
    t.start()
    return Response(status=200)
    

def process():
    print("processing")
    processor.Impr.compute()
    
    
@app.route("/numprocessed", methods=["GET"])
def numberProcessed():
    return str(processor.num_processed)
    
    
@app.route('/getprocessed/<pk>')
def getimage(pk):
    filename = processor.sr_path +'/' + pk + '_sr.jpg'
    return send_file(filename, mimetype='image/jpg')

@app.route('/clear')
def clean():
    for filename in os.listdir(processor.sr_path):
        file_path = os.path.join(processor.sr_path, filename)
        os.remove(file_path)
    for filename in os.listdir(processor.lr_path):
        file_path = os.path.join(processor.lr_path, filename)
        os.remove(file_path)
    processor.num_processed = 0
    return Response(status=200)
    

app.run(host="0.0.0.0", port=8000)