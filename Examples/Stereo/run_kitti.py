import sys
import os
import subprocess
import threading
import signal
import re
import pandas as pd
import time

def server(port,event):
    print("Server started")
    args=["./stereo_kitti" ,'../../Vocabulary/ORBvoc.txt' ,'KITTI00-02.yaml' ,'server']
    child_process=subprocess.Popen(args,stdin=subprocess.PIPE)
    inp='127.0.0.1\n'+str(port)+'\n'+str(port+1)+'\n'+str(port+2)+'\n'
    input=['127.0.0.1\n',str(port)+'\n',str(port+1)+'\n',str(port+2)+'\n']
    count=0
    print('writing',input[0])
    child_process.stdin.write(bytes(input[0],'utf-8'))
    print('writing',input[1])
    child_process.stdin.write(bytes(input[1],'utf-8'))
    print('writing',input[2])
    child_process.stdin.write(bytes(input[2],'utf-8'))
    print('writing',input[3])   
    child_process.stdin.write(bytes(input[3],'utf-8'))
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
    child_process=subprocess.Popen(args,stdin=subprocess.PIPE)
    inp='127.0.0.1'+'\n'+'127.0.0.1'+'\n'+str(port)+'\n'+str(port)+'\n'+str(port+1)+'\n'+str(port+1)+'\n'+str(port+2)+'\n'+str(port+2)+'\n'
    child_process.communicate(input=bytes(inp,'utf-8'))
    child_process.wait()
    print('client finished')
    return

def run_evo(gt,traj='KeyFrameTrajectory_TUM_Format1.txt'):
    evo_proc=subprocess.Popen(['evo_ape','tum',gt,traj,'-va'],stdout=subprocess.PIPE)
    out,err=evo_proc.communicate()
    print(err)
    # print(out)
    outp=out.decode('utf-8')
    out=out.decode('utf-8').split('\n')
    # print('comapred',out[-15])
    # print('b4 proc',out)
    if len(out)<10:
        print('ERROR IN EVO')
        print(out)
        out=['error in evo' for i in range(8)]
        return out
    pose_pair=int(out[-15].split(' ')[1])
    print('pose pair',pose_pair)
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
    return out[-7:]+[pose_pair]

if len(sys.argv) != 5:
    print("Usage: python run_kitti.py <dataset> <portStart> <gt> <run_times>")
    sys.exit(1)

dataset = sys.argv[1]
portStart = int(sys.argv[2])
index=['max','mean','median','min','rmse','sse','std','compare pose pairs','numKFS']
df=pd.DataFrame(columns=index)
print(df)

runs=int(sys.argv[4])


# res=run_evo(sys.argv[3])

for i in range(runs):
    

    event=threading.Event()
    server_thread = threading.Thread(target=server, args=(portStart,event,))
    server_thread.start()

    time.sleep(1)

    client_thread = threading.Thread(target=client, args=(portStart,dataset,event,))
    client_thread.start()

    # print('waiting for threads to finish')
    client_thread.join()
    # print('client joined')
    event.set()
    server_thread.join()
    print('server joined')
    gt=sys.argv[3]

    evo_res=run_evo(gt)
    print('res',evo_res)

    file=open('KeyFrameTrajectory_TUM_Format1.txt','r')
    lines=file.readlines()
    num=len(lines)
    file.close()

    print(evo_res+[num])
    df.loc[-1]=evo_res+[num]
    df.index = df.index + 1

    portStart+=3

    print(df.head())

df_t=df.transpose()
df_t.to_csv('results.csv',sep='\t')

proc=subprocess.Popen(['cat','results.csv'])

proc.wait()
os.remove('results.csv')



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


