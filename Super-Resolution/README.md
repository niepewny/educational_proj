# Super-Resolution

This the task was to put the image processing program using the architecture from **[this repo](https://github.com/yoon28/pseudo-sr)** on the local server and create and create an app in c++ that would receive photos from the user, rend it to the server, receive the processed ones back.

It consists of two parts: Server, created with python and User_app, created with c++.

The User_app was created with Qt and to use it, user should run **AI_upscaling.pro** file with QtCreator.

To start server, user should run **server.py** file. It requires few libraries, but user shuld be informed about them and instalation should be easy.

The AI net appeared to be extremaly demanded and, so processing big photos is time-consuming and it requires a lot of memory. That's why dy default it doesn't use CUDA, but the user can change it by changing rant to *torch.device("cuda")* in **processor.py** script (not recomended). 

Due to the fact, that the project has only presentation meaning, there are sample low resolution photos (16x16px) in **sample_pictures** dictionary to speed up an app test.

# required files:
 ### [net](https://drive.google.com/file/d/1oQEbu9Bszh5ZQwIPoyMwidSI7AY1UHwg/view?usp=sharing)
 - it should be pasted in the ***server/DATASET/nets*** directory

 # warnings
- The process of learning was too time-consuming, so it was stopped after 1 epoch (24h of learning). In result, the output is not impressive at all. In the future I may train it better using Google Colab.
- The project still contains some bugs. in case of unplanned exiting, it should be oppened, closed and oppened again, as closing invokes destructor, which should remove unwanted files from previous runs