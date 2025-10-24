#!/usr/bin/env python3

import sys
import signal
import re
import json
from termcolor import colored

launch_script = 'launch/harpia_launch.py'

signal.signal(signal.SIGPIPE, signal.SIG_DFL)

def printb(arg):
    print(json.dumps(arg, indent=2))

pattern1 = re.compile(r'\[(.*?)\] \[(.*?)\] \[(.*?)\] \[(.*?)\]: (.*)')
pattern2 = re.compile(r'\[(.*?)\] \[(.*?)\]: (.*)')
pattern3 = re.compile(r'\[(.*?)\] (.*)')

logs = []
not_recognized = []
first_time = 0
got_first_time = False

def read_configs():
    try:
        with open('process_output_config.json') as f:
            print(json.load(f))
    except FileNotFoundError:
        print('No config file found, creating...')
        write_configs()

def write_configs():
    try:
        with open(launch_script) as lf:
            launch_file = lf.read()
    except FileNotFoundError:
        print(f"Launch script '{launch_script}' not found")
        return
    
    try:
        idx = launch_file.index('nodes_to_add')
        launch_file = launch_file[idx:]
        idx = launch_file.index('[')
        launch_file = launch_file[idx:]
        idx = 0
        count = 0
        while True:
            if launch_file[idx] == '[':
                count += 1
            if launch_file[idx] == ']':
                count -= 1
                
    except Exception as e:
        print(f"Couldn't find the nodes in launch script, error: {e}")
        return
    print(launch_file)

    # with open('process_output_config.json', 'w') as f:
    #     json.dump({
    #         'print_all': False,
    #         'nodes_to_print': []
    #     }, f, indent=2)
    # read_configs()

# for line in str.split('\n'):
def process_new_line(line):
    global got_first_time, first_time
    if not got_first_time:
        if len(logs) > 0:
            if logs[-1]['time_in_seconds'] is not None:
                first_time = logs[-1]['time_in_seconds']
                logs[-1]['time_in_seconds'] = 0
                got_first_time = True

    match1 = pattern1.match(line)
    if match1:
        file_name, type_of_message, time_in_seconds, node_name, message = match1.groups()
        time_in_seconds = float(time_in_seconds)-first_time
        # print(f"File Name: {file_name}")
        # print(f"Type of Message: {type_of_message}")
        # print(f"Time in Seconds: {time_in_seconds}")
        # print(f"Node name: {node_name}")
        # print(f"Message: {message}")
        handle_new_log({
            'type': 1,
            'file_name': file_name,
            'type_of_message': type_of_message,
            'time_in_seconds': time_in_seconds,
            'node_name': node_name,
            'message': message,
            'test': False
        })
        return
    
    match2 = pattern2.match(line)
    if match2:
        type_of_message, file_name, message = match2.groups()
        # print(f"Type of Message: {type_of_message}")
        # print(f"Node name: {file_name}")
        # print(f"Message: {message}")
        # print(f'[{type_of_message}] [{file_name}]: {message}\n{line}\n')

        handle_new_log({
            'type': 2,
            'file_name': file_name,
            'type_of_message': type_of_message,
            'time_in_seconds': None,
            'node_name': None,
            'message': message,
            'test': False
        })
        return

    match3 = pattern3.match(line)
    if match3:
        file_name, message = match3.groups()
        
        # print(file_name)
        # print(logs[-1])
        # print('')

        if file_name == logs[-1]['file_name']:
            logs[-1]['message'] += '\n'+message
            logs[-1]['test'] = True
            handle_log_continuation(message)
            return

    handle_not_recognized(line)

node_colors = [
    ('red',           []),
    ('green',         []),
    ('yellow',        []),
    ('blue',          []),
    ('magenta',       []),
    ('cyan',          []),
    # ('white',         []),
    ('light_red',     []),
    # ('light_grey',    []),
    # ('dark_grey',     []),
    ('light_green',   ['underline']),
    ('light_yellow',  ['underline']),
    ('light_blue',    ['underline']),
    ('light_magenta', ['underline']),
    ('light_cyan',    ['underline']),
    ('light_grey',    ['underline']),
    ('dark_grey',     ['underline']),
]
color_index = 0
def next_color():
    global color_index
    r = node_colors[color_index]
    color_index = (color_index-1)%len(node_colors)
    return r
nodes = {}

def handle_new_log(log):
    logs.append(log)
    
    # if not log['test']:
    #     continue

    # if log['node_name'] != 'plansys_interface':
    #     continue

    # print(colored(json.dumps(log, indent=2), 'green'))

    if log['file_name'] not in nodes:
        nodes[log['file_name']] = next_color()


    msg_attrs = []
    msg_color = "blue"

    if log['type_of_message'] == "INFO":
        msg_color = "white"
    elif log['type_of_message'] == "ERROR":
        msg_color = "red"
    elif log['type_of_message'] == "WARN":
        msg_color = "yellow"

    str = ""

    if log['file_name'] is not None:
        str += colored('[', 'blue')+colored(f"{log['file_name']}", nodes[log['file_name']][0])+colored('] ', 'blue')
    # if log['node_name'] is not None:
    #     str += colored(f"[{log['node_name']}] ", nodes[log['file_name']][0])
    # if log['type_of_message'] is not None:
        # str += colored(f"[{log['type_of_message']}] ", 'blue') 
    if log['time_in_seconds'] is not None:
        str += colored(f"[{log['time_in_seconds']:.2f}] ", 'blue')

    # str = str[0:-1]

    str += colored(f"{log['message']}", color=msg_color,attrs=msg_attrs)

    print(str)

def handle_log_continuation(message):
    lastLog = logs[-1]

    msg_attrs = []
    msg_color = "blue"

    if lastLog['type_of_message'] == "INFO":
        msg_color = "white"
    elif lastLog['type_of_message'] == "ERROR":
        msg_color = "red"
    elif lastLog['type_of_message'] == "WARN":
        msg_color = "yellow"

    str = ""
    if lastLog['file_name'] is not None:
        str = colored('[', 'blue')+colored(f"{lastLog['file_name']}", nodes[lastLog['file_name']][0])+colored('] ', 'blue')
    
    str += colored(f"{message}", color=msg_color,attrs=msg_attrs)

    print(str)


def handle_not_recognized(line):
    line = line.replace('\n', '')
    not_recognized.append(line)
    print(colored('Not recognized: ', 'red')+colored(f" {line}", 'blue'))



# read_configs()
# while True:
#     pass

try:
    for line in sys.stdin:
        process_new_line(line)
except KeyboardInterrupt:
    print(colored(' Interrupted', 'red'))