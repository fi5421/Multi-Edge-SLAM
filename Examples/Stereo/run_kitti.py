import sys
import os
import subprocess
import threading
import signal
import re
import pandas as pd
import time
import shutil
import logging
import json
# import progressbar as Bar
import matplotlib.pyplot as plt

global track_lost
track_lost=False

def plotKeyFrameGeneration(resultsPath, groundtruthPath):
    # Reading groundtruth
    groundtruth = pd.read_csv(groundtruthPath, delim_whitespace=True, header=None)

    # Reading configuraiton parameters
    # config = pd.read_csv(f'results/{resultsPath}/{resultsPath}.txt', delimiter='\t', index_col=0, header=None)
    # syncFrame = config.iloc[5][1]
    # switchFrame = config.iloc[4][1]
    f=open(f'results/{resultsPath}/{resultsPath}.txt','r')
    data=json.load(f)   
    syncFrame = data['sync']
    switchFrame = data['switch']

    # Experiemnt can either involve both state migration and handover, just handover or neither
    if syncFrame == "No" and switchFrame == "No":
        syncTimestamp = -1
        handoverTimestamp = -1
    elif syncFrame == "No" and switchFrame != "No":
        syncTimestamp = -1
        handoverTimestamp = groundtruth.iloc[int(switchFrame)-1, 0]
    else:
        syncTimestamp = groundtruth.iloc[int(syncFrame)-1, 0]
        handoverTimestamp = groundtruth.iloc[int(switchFrame)-1, 0]

    # Average KeyFrame Generation
    trajectories = []
    maxSeenTimestamp = 0

    # Finding largest seen timestamp across all trajectories
    for root, dirs, files in os.walk(f'results/{resultsPath}/traj'):
        for file in files:
            if file.endswith(".txt"):
                file_path = os.path.join(root, file)

                trajectory = (pd.read_csv(file_path, sep='\s+', header=None)).iloc[:, 0]
                trajectory = trajectory - trajectory[0]     # normalizing by offsetting from first timestamp
                trajectory = trajectory.astype(int)         # truncating decimal
                trajectory = trajectory.sort_values()
                trajectories.append(trajectory)

                # Calculating metadata
                keyFrameGeneration = trajectory.value_counts().sort_index()
                timestamps = [value for value, count in keyFrameGeneration.items() for _ in range(count)]             
                timeTaken = max(timestamps)

                if timeTaken > maxSeenTimestamp:
                    maxSeenTimestamp = timeTaken

    # Computing average distribution across all trajectories
    firstIteration = True
    for trajectory in trajectories:
        keyFrameGeneration = trajectory.value_counts().sort_index()
        keyFrameGeneration = keyFrameGeneration.reindex(range(maxSeenTimestamp + 2), fill_value=0)
        if firstIteration:
            avg_keyFrameGeneration = keyFrameGeneration
            firstIteration = False
        else:
            avg_keyFrameGeneration = avg_keyFrameGeneration + keyFrameGeneration
            avg_keyFrameGeneration = avg_keyFrameGeneration/2

    # Normalizing sync and handover timestamps by offsetting from first timestamp
    handoverTimestamp = handoverTimestamp - trajectories[0][0]
    syncTimestamp = syncTimestamp - trajectories[0][0]
    
    # Dynamically scalable plot
    peakGeneration = max(avg_keyFrameGeneration)    # maximum keyFrames generated in a second
    plt.figure(figsize=((0.25*maxSeenTimestamp), peakGeneration))
    
    # Plotting KeyFrame Generation across Time
    plt.bar(avg_keyFrameGeneration.index, avg_keyFrameGeneration.values, color='black', width=1)

    # Plotting State Migration Region (if any)
    if syncTimestamp == -1 and handoverTimestamp == -1:
        pass
    elif syncTimestamp == -1 and handoverTimestamp != -1:
        plt.axvline(x=handoverTimestamp, color='red', linestyle='--', linewidth=2, label='Handover (No State Migration)')
        plt.legend()
    else:
        plt.axvline(x=syncTimestamp, color='blue', linestyle='--', linewidth=2, label='Start Sync Protocol')
        plt.axvline(x=handoverTimestamp, color='red', linestyle='--', linewidth=2, label='Stop Sync and Handover')
        plt.axvspan(syncTimestamp, handoverTimestamp, color='green', alpha=0.3, label='State Migration Region')
        plt.legend()

    # Set labels and title
    plt.xlabel('Time (seconds)')
    plt.ylabel('Average Number of KeyFrames Generated')
    plt.title('Average KeyFrame Generation Across Time')

    # Customizing  appearance
    plt.grid(axis='y', linestyle='--')
    plt.xticks(keyFrameGeneration.index)
    plt.tight_layout()
    
    # Saving plot
    figFilename = resultsPath.replace('.txt', '') + '.png'
    plt.savefig(os.path.join(f'results/{resultsPath}', figFilename))

def server(port,event,server='server'):
    print("Server started")
    args=["./stereo_kitti" ,'../../Vocabulary/ORBvoc.txt' ,'KITTI00-02.yaml' ,server]
    child_process=subprocess.Popen(args,stdin=subprocess.PIPE)
    inp='127.0.0.1\n'+str(port)+'\n'+str(port+1)+'\n'+str(port+2)+'\n'

    subset_port=port-1
    if(server=='server2'):
        subset_port=port-2
    input=['127.0.0.1\n',str(port)+'\n',str(port+1)+'\n',str(port+2)+'\n',str(port-1),str(subset_port)+'\n']
    count=0
    # print('writing',input[0])
    # child_process.stdin.write(bytes(input[0],'utf-8'))
    # print('writing',str(subset_port)+'\n'+input[1])/
    print('writing',input[1])
    # child_process.stdin.write(bytes(input[-1],'utf-8'))
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
    # out=child_process.stdout.readlines()
    # out=[i.decode('utf-8') for i in out]
    # print('printing out')
    # # print(out)
    # print(server)
    # for i in out:
    #     print(i[:-1])
    # for i in out:
    #     if 'TRACKING LOST BEFORE HANDOVER' in i:
    #         print('fount',i)
    #         global track_lost
    #         track_lost=True
    #         break

    child_process.wait()
    print('signal sent2')

def client(port,dataset,event):
    print("client started")
    args=["./stereo_kitti" ,'../../Vocabulary/ORBvoc.txt' ,'KITTI00-02.yaml' ,'client',dataset]
    print('making subporcess')
    child_process=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
    print('subprocess made')
    # inp='127.0.0.1'+'\n'+'127.0.0.1'+'\n'+str(port)+'\n'+str(port)+'\n'+str(port+1)+'\n'+str(port+1)+'\n'+str(port+2)+'\n'+str(port+2)+'\n'
    inp=str(port)+'\n'+'1\n'
    out,err=child_process.communicate(input=bytes(inp,'utf-8'))
    out=out.decode('utf-8').split('\n')
    for i in out:
        print(i)
        if "TRACKING LOST BEFORE HANDOVER" in i:
            global track_lost
            track_lost=True
            break
    # out=[i.decode('utf-8') for i in out]
    # for i in out:
    #     print(i[:-1])
    #     if 'TRACKING LOST BEFORE HANDOVER' in i:
    #         print('fount',i)
    #         global track_lost
    #         track_lost=True
    #         break
   
    child_process.wait()
    return
    print('client returned')
    return child_process
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
        print('ERROR IN EVO')
        out=['error in evo' for i in range(8)]
        return out
    num_pose=int(out[-15].split(' ')[1])
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
    return out[-7:]+[num_pose]

# time_text=time.strftime("%Y%m%d-%H%M%S")
if len(sys.argv)<6:
    print('Usage1: python3 run_kitti.py dataset port_start gt runs comp_gt optional: branch switch sync time_stamp')
    exit()


print(sys.argv)
if len(sys.argv)==10:
    time_text=sys.argv[9]
else:
    time_text=time.strftime("%d-%m-%Y-%H:%M:%S")


# time_text=time.strftime("%d-%m-%Y-%H:%M:%S")
dir='results/'+time_text+'/'
# logging.basicConfig(filename=dir+'run_kitti.log',level=logging.DEBUG)



dataset = sys.argv[1]
portStart = int(sys.argv[2])
index=['max','mean','median','min','rmse','sse','std','num pose','numKFS1','numKFS2','totalKFS']
df=pd.DataFrame(columns=index)
# logging.debug(df)

runs=int(sys.argv[4])

# proc=subprocess.Popen(['git','branch'])
# proc.wait()
if len(sys.argv)>6:
    branch=sys.argv[6]
else:
    branch=input('enter branch name: ')
# branch=input('enter branch name: ')


gt=sys.argv[3].split('/')[-1]
comp_gt=sys.argv[5]


# logging.debug('\t'+ dataset.to_string().replace('\n', '\n\t'))
datasetl=dataset.split('/')[-3:-1]
datasetl='/'.join(datasetl)



if len(sys.argv)>6:
    switch=sys.argv[7]
else:
    switch=input('swith frame:')

if len(sys.argv)>6:
    sync=sys.argv[8]
else:
    sync=input('sync frame:')

    
text_l=[f'branch\t{branch}',f'dataset\t{datasetl}',f'gt\t{gt}',f'runs\t{runs}',f'switch\t{switch}',f'sync\t{sync}',f'time\t{time_text}']
if not os.path.exists(dir):
    os.makedirs(dir)
    print("making",dir)

dir_traj=dir+'traj/'
if not os.path.exists(dir_traj):
    os.makedirs(dir_traj)
logging.basicConfig(level=logging.DEBUG,filename=dir+'run_kitti.log',filemode='w')

text_d={'branch':branch,'dataset':datasetl,'gt':gt,'runs':runs,'switch':switch,'sync':sync,'time':time_text}

# dir='metadata'
# if not os.path.exists(dir):
#     os.makedirs(dir)

run_count=0
error_count=0
track_b4=0

# class SlowBar(Bar):
#     max=runs
#     message="Slamming"
#     suffix = '%(percent).1f%%  '
#     @property
#     def remaining_minutes(self):
#         return self.eta // 60

# bar = Bar('SLAMing', max=runs,fill="F", suffix='%(index)d/%(max)d - %(percent).1f%% - %(eta)ds')
# filename_csv=input('Enter filename for csv: ')


print("switch", switch, "sync", sync, 'branch', branch,'time_stamp',time_text)
# exit()
gt=sys.argv[3]

while run_count<runs:
    try:
    
        track_lost=False

        event=threading.Event()
        server_thread = threading.Thread(target=server, args=(portStart,event,))
        server_thread.start()
        
        # time.sleep(1)

        server2_thread = threading.Thread(target=server, args=(portStart+1,event,'server2'))
        server2_thread.start()

        time.sleep(5)

        client_thread = threading.Thread(target=client, args=(portStart,dataset,event,))
        client_thread.start()

        # loggin.debug('waiting for threads to finish')
        logging.debug('waiting for client to finish')
        client_thread.join()
        logging.debug('client joined')
        event.set()
        logging.debug('waiting for server to finish')
        server_thread.join()
        server2_thread.join()
        time.sleep(2)
        logging.debug('server joined')
        # time.sleep(2)

        


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


        try:
            evo_res=run_evo(gt,traj='KeyFrameTrajectory_TUM_Format_combined.txt')
            logging.debug("evo result")
            logging.debug(str(evo_res))
        except:
            evo_res=['error in evo' for i in range(8)]
        

        if track_lost:
            track_b4+=1

        if(evo_res[0]=='error in evo'):
            error_count+=1
            logging.debug('ERROR IN EVO')
            t=f'Run Count:\\t{run_count}\\n',f'Error Count:\\t{error_count}\\n',f'Track Lost Before Handover:\\t{track_b4}\\n'
            logging.debug('Run Count:\t%d\n Error Count:\t%d\n Track Lost Before Handover:\t%d\n',run_count,error_count,track_b4)

            portStart+=10
            text_d['Run Count']=run_count
            text_d['Error Count']=error_count
            text_d['Track Lost Before Handover']=track_b4
            if not os.path.exists(dir):
                os.makedirs(dir)

            json_obj=json.dumps(text_d, indent=4)

            # file=open(dir+'/'+time_text+'.txt','w')
            # file.writelines(text_l)
            # file.close()

            with open(dir+'/'+time_text+'.txt','w') as file:
                file.write(json_obj)
                file.close()

            continue
        traj_path=f"{dir_traj}{run_count}"

        if not os.path.exists(traj_path):
            os.makedirs(traj_path)
        logging.debug(traj_path)

        shutil.copy('KeyFrameTrajectory_TUM_Format1.txt',f"{traj_path}/KeyFrameTrajectory_TUM_Format1.txt")
        shutil.copy('KeyFrameTrajectory_TUM_Format2.txt',f"{traj_path}/KeyFrameTrajectory_TUM_Format2.txt")
        shutil.copy('KeyFrameTrajectory_TUM_Format_combined.txt',f"{traj_path}/KeyFrameTrajectory_TUM_Format_combined.txt")
        run_count+=1

        # logging.debug(evo_res+[num])
        df.loc[-1]=evo_res+[num1,num2,num1+num2]
        df.index = df.index + 1
        t=f'Run Count:\t{run_count}\n',f'Error Count:\t{error_count}\n',f'Track Lost Before Handover:\t{track_b4}\n'
        logging.debug('Run Count:\t%d\n Error Count:\t%d\n Track Lost Before Handover:\t%d\n',run_count,error_count,track_b4)

        portStart+=10

        text_d['Run Count']=run_count
        text_d['Error Count']=error_count
        text_d['Track Lost Before Handover']=track_b4

        text_l=[i+'\n' for i in text_l]
        # dir='metadata'
        if not os.path.exists(dir):
            os.makedirs(dir)

        json_obj=json.dumps(text_d, indent=4)

        # file=open(dir+'/'+time_text+'.txt','w')
        # file.writelines(text_l)
        # file.close()

        with open(dir+'/'+time_text+'.txt','w') as file:
            file.write(json_obj)
            file.close()

        logging.debug(df.to_string())



        df_t=df.transpose()
        df_t.to_csv(dir+'/'+time_text+'.csv',sep='\t')
        # bar.next()
    except KeyboardInterrupt:
        print("KeyboardInterrupt has been caught.")
        exit=input("input y to exit")
        if exit=='y':
            break
        else:
            portStart+=10
            continue

        


for i in text_l:
    logging.debug(i[:-1])

# logging.debug('SAVED TO :',time_text+'.csv')
logging.debug('SAVED TO : %s.csv',time_text)

dir='results'
proc=subprocess.Popen(['cat',dir+'/'+time_text+'/'+time_text+'.csv'])

proc.wait()

plotKeyFrameGeneration(time_text, comp_gt)

# bar.finish()


# os.remove('tab.csv')




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


