import sys
import os
import subprocess
import threading
import signal
import re
import pandas as pd
import time

def server(port,event,server='server'):
    print("Server started")
    args=["./stereo_kitti" ,'../../Vocabulary/ORBvoc.txt' ,'KITTI00-02.yaml' ,server]
    child_process=subprocess.Popen(args,stdin=subprocess.PIPE)
    inp='127.0.0.1\n'+str(port)+'\n'+str(port+1)+'\n'+str(port+2)+'\n'
    input=['127.0.0.1\n',str(port)+'\n',str(port+1)+'\n',str(port+2)+'\n']
    count=0
    # print('writing',input[0])
    # child_process.stdin.write(bytes(input[0],'utf-8'))
    print('writing',input[1])
    child_process.stdin.write(bytes(input[1],'utf-8'))
    # print('writing',input[2])
    # child_process.stdin.write(bytes(input[2],'utf-8'))
    # print('writing',input[3])   
    # child_process.stdin.write(bytes(input[3],'utf-8'))
    child_process.stdin.flush()
    child_process.stdin.close()
    print('here2')
    print('event waiting')
    event.wait()
    print('signal sent1')
    child_process.send_signal(signal.SIGINT)
    child_process.wait()
    print('signal sent2')

def client(port,dataset,event):
    print("client started")
    args=["./stereo_kitti" ,'../../Vocabulary/ORBvoc.txt' ,'KITTI00-02.yaml' ,'client',dataset]
    print('making subporcess')
    child_process=subprocess.Popen(args,stdin=subprocess.PIPE)
    print('subprocess made')
    # inp='127.0.0.1'+'\n'+'127.0.0.1'+'\n'+str(port)+'\n'+str(port)+'\n'+str(port+1)+'\n'+str(port+1)+'\n'+str(port+2)+'\n'+str(port+2)+'\n'
    inp=str(port)+'\n'+'1\n'
    child_process.communicate(input=bytes(inp,'utf-8'))
    child_process.wait()
    print('client finished')
    return

def run_evo(gt,traj='KeyFrameTrajectory_TUM_Format1.txt'):
    evo_proc=subprocess.Popen(['evo_ape','tum',gt,traj,'-va'],stdout=subprocess.PIPE)
    out,err=evo_proc.communicate()
    print(err)
    outl=out.decode('utf-8')
    print(outl)
    out=out.decode('utf-8').split('\n')
    # print('b4 proc',out)
    if len(out)<10:
        out=['error in evo' for i in range(7)]
        return out
    out=[i for i in out if i!='']
    for i in range(len(out)):
        
        # string=string.replace('\t',' ')
        string = re.sub('[^0-9.\t]', '', out[i])
        string=string.replace('\t',"")
        try:
            out[i]=float(string)
        except:
            out[i]=0
    # print(out[-7:])
    return out[-7:]

if len(sys.argv) != 5:
    print("Usage: python run_kitti.py <dataset> <portStart> <gt> <run_times>")
    sys.exit(1)

dataset = sys.argv[1]
portStart = int(sys.argv[2])
index=['max','mean','median','min','rmse','sse','std','numKFS1','numKFS2','totalKFS']
df=pd.DataFrame(columns=index)
print(df)

runs=int(sys.argv[4])

# filename_csv=input('Enter filename for csv: ')

for i in range(runs):
    

    event=threading.Event()
    server_thread = threading.Thread(target=server, args=(portStart,event,))
    server_thread.start()

    server2_thread = threading.Thread(target=server, args=(portStart+1,event,'server2'))
    server2_thread.start()

    time.sleep(1)

    client_thread = threading.Thread(target=client, args=(portStart,dataset,event,))
    client_thread.start()

    # print('waiting for threads to finish')
    client_thread.join()
    # print('client joined')
    event.set()
    server_thread.join()
    server2_thread.join()
    print('server joined')
    gt=sys.argv[3]


    file=open('KeyFrameTrajectory_TUM_Format1.txt','r')
    lines1=file.readlines()
    num1=len(lines1)
    file.close()

    file=open('KeyFrameTrajectory_TUM_Format2.txt','r')
    lines2=file.readlines()
    num2=len(lines2)
    file.close()

    file=open('KeyFrameTrajectory_TUM_Format_combined.txt','w')
    file.writelines(lines1+lines2)
    file.close()

    evo_res=run_evo(gt,traj='KeyFrameTrajectory_TUM_Format_combined.txt')
    print('res',evo_res)

    # print(evo_res+[num])
    df.loc[-1]=evo_res+[num1,num2,num1+num2]
    df.index = df.index + 1

    portStart+=6

    print(df)

# pd.set_option('display.colhedaer)
df_t=df.transpose()
print(df_t)
df_t.to_csv('tab.csv',sep='\t')

proc=subprocess.Popen(['cat','tab.csv'],stdout=subprocess.PIPE)
out=proc.communicate()
# print(out)
out=out[0].decode('utf-8')
print(out)

os.remove('tab.csv')




# evo_proc=subprocess.Popen(['evo_ape','tum',gt,'KeyFrameTrajectory_TUM_Format1.txt','-va'],stdout=subprocess.PIPE)
# out,err=evo_proc.communicate()
# print(err)
# out=out.decode('utf-8').split('\n')
# out=[i for i in out if i!='']
# for i in range(len(out)):
    
#     # string=string.replace('\t',' ')
#     string = re.sub('[^0-9.\t]', '', out[i])
#     string=string.replace('\t',"")
#     try:
#         out[i]=float(string)
#     except:
#         out[i]=0
# print(out[-7:])


# evo_proc.wait()


