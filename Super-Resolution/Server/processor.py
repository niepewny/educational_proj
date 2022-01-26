import sre_compile
from models import face_model as Model
import torch, os, glob
from yacs.config import CfgNode
from tools.utils import save_tensor_image
from tools.pseudo_face_data import faces_data
class Impr():


    global net_path, lr_path, sr_path, CFG, num_processed
    CFG = CfgNode.load_cfg(open("./configs/faces.yaml", "rb"))
    net_path = os.path.join(CFG.DATA.FOLDER,"nets/*.pth")
    lr_path=os.path.join(CFG.DATA.FOLDER, "lowr")
    sr_path = os.path.join(CFG.EXP.OUT_DIR, "imgs")
    num_processed = 0


    def compute(self):

        if not os.path.exists(lr_path):
            os.makedirs(lr_path)
        if not os.path.exists(sr_path):
            os.makedirs(sr_path)

        rank = torch.device('cpu') #for high GPU memory "cuda"
        net = Model.Face_Model(rank, CFG, 0)
        net.net_load(net_path)
        net.mode_selector("eval")

        global num_processed
        change = faces_data(lr_path,  b_train=False, shuffle=False, img_range=CFG.DATA.IMG_RANGE, rgb=CFG.DATA.RGB)
        files = glob.glob(lr_path)

        num_files = len(next(os.walk(lr_path))[2])
        for file in range(num_files):
            lr = change[file]["lr"].unsqueeze(0).to(rank)
            y, sr, _  = net.test_sample(lr)
            image_path = os.path.join(sr_path,(str(file) + "_sr.jpg"))
            save_tensor_image(image_path, sr, CFG.DATA.IMG_RANGE, CFG.DATA.RGB)
            num_processed += 1

        print('its done')