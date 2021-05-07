import keyboard, subprocess, os, signal, cv2
from multiprocessing import Process

processes = []

def run(*args):
    print(f'Running {" ".join(args)} with ID: {os.getpid()}')
    subprocess.run(args)

def senario(s, *args):
    global processes
    for process in processes:
        print(f'Killing process {process.pid}')
        process.kill()
        #os.kill(process.pid, signal.SIGKILL)
        #os.waitpid(process.)
    if s == 1:
        controller = Process(target=run, args=['python3', 'robotController.py', '-s'])
        serial = Process(target=run, args=['python3', 'serial_esp.py'])
        processes = [controller, serial]
    elif s == 2:
        processes = []
    for process in processes:
        print(f'Starting process {process}')
        process.start()

        #(['python3', 'keyboard_drive.py'])

keyboard.add_hotkey('1', senario, args=[1])
keyboard.add_hotkey('2', senario, args=[2])
keyboard.wait()
