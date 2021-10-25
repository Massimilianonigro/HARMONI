#!/usr/bin/env python3
import rospy
import subprocess
from libnmap.process import NmapProcess
from libnmap.parser import NmapParser
from time import sleep
from std_msgs.msg import Bool


shell_command = "hostname -I"
process = subprocess.check_output(shell_command, shell=True)
process = process.decode("utf-8") 
ips = process.split(" ")
for ip in ips:
    if "192" in ip:
        local_url = ip
mac = '00:d0:ca:01:f8:53'
ip =local_url

print(f'Here is your local ip {ip}')
nm = NmapProcess(f'{ip}/24', options="-sP")
nm.run_background()

while nm.is_running():
    print(f'Nmap Scan running: ETC: {nm.etc} DONE: {nm.progress}%')
    sleep(2)
nmap_report = NmapParser.parse(nm.stdout)

res = next(filter(lambda n:n.mac == mac.strip().upper(), filter(lambda host:host.is_up(), nmap_report.hosts)), None)
if res == None:
    print("Host is down or Mac address not exist")
else:
    print(f'\nMAC: {mac} with IP {res.address}')
    rospy.set_param("robot_ip", res.address)
    rospy.set_param("local_ip", local_url)
"""
cmd = 'arp -a | findstr "ff-ff-ff-ff-ff-ff" '
returned_output = subprocess.check_output((cmd),shell=True,stderr=subprocess.STDOUT)
print(returned_output)
parse=str(returned_output).split(' ',1)
ip=parse[1].split(' ')
print(ip[1])                                                                                   
"""
