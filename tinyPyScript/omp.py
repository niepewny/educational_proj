import numpy as np
import matplotlib.pyplot as plt
import os

#sile ustawilem na 0.5 - 5.5 kN
X_data = np.arange(0.5, 6, 0.5)
x_label = 'F [kN]'

#ilość plikow csv
amount_of_files = 15

for i in range (amount_of_files):
    sript_dir = os.path.dirname(__file__)

    #folder z plikami csv
    path_to_file = os.path.join(sript_dir,'data\P' + str(i+1) + '.csv' )
    data_array = np.genfromtxt(path_to_file, dtype=float, delimiter=',', skip_header=1)

    
    for j in range (5,9):
        plt.figure()
        Y_data = data_array[:,j]
        plt.plot(X_data, Y_data)
        plt.grid(True)

        if(j == 5):
            title = 'Przemieszczenia u'
            y_label = 'u [mm]'

        elif(j == 6):
            title = 'Przemieszczenia v'
            y_label = 'v [mm]'

        elif(j == 7):
            title = 'Odksztalcenia Exx'
            y_label = 'Exx'
        
        else:
            title = 'Odksztalcenia Eyy'
            y_label = 'Eyy'


        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)

        #folder *E:/python/Plots/...* folder wyjsciowy
        plt.savefig(os.path.join(sript_dir,'plots\P' + str(i+1) +str(i+1)+"."+str(j-4)+".png"))
        

