#!/usr/bin/env python
# coding=utf-8
import requests
import json

# register this device on ROBxTASK cloud platform with skill implementation documentation
# INFO: needs Basic Authentication: USER: "devr" + PW: "DevReg!robXtask"
registration_endpoint = 'https://robxtask.salzburgresearch.at/robxtask/registration-service/v3/api-docs?group=all/device' 

#---------------------------------------------------------------------------------------------
# loadRegistrationFile
#---------------------------------------------------------------------------------------------
def loadRegistrationFile():

    with open('/home/rosdev/arti_ws/src/rxt_skills_chasi/scripts/python/CHASI_ros_aas_registration/CHASI_registration_file.json', 'r') as dt_file:
        dt_data = json.load(dt_file) 
        # TODO
    
    return dt_data


# -------------------------------------------------------------------------------------------
# uploadAAS (upload full settings with all entries)
# -------------------------------------------------------------------------------------------
def uploadAAS(aas):
    
    try:
        r = requests.post(registration_endpoint, timeout=5, json=aas, auth=('devr', 'DevReg\!robXtask'))
        headers = {'Content-type': 'application/json'}      
            
        if r.ok:
            print("Description uploaded succesfullly")
        else:
            print("--------------------------")
            print("Error in server response: " + str(r.status_code))
            print("--------------------------")
                        
    except requests.exceptions.RequestException as e:
        print (e)
    
    
    
