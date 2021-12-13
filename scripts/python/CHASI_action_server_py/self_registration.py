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

    with open('/home/rosdev/arti_ws/src/rxt_skills_chasi/scripts/python/CHASI_ros_aas_registration/CHASI_registration_file.json', 'r') as json_file:
        json_data = json.load(json_file) 
    
    return json_data


# -------------------------------------------------------------------------------------------
# uploadAAS (upload full settings with all entries)
# -------------------------------------------------------------------------------------------
def uploadAAS(aas):
    
    try:
        headers = {'Content-type': 'application/json'}    
        r_get = requests.get(registration_endpoint + '/00:e0:4c:68:02:30', timeout=5, json=aas, headers=headers, auth=('devr', 'DevReg\!robXtask'))

        if r_get.ok:
            r_add = requests.put(registration_endpoint, timeout=5, json=aas, headers=headers, auth=('devr', 'DevReg\!robXtask')) 
        else:
            r_add = requests.post(registration_endpoint, timeout=5, json=aas, headers=headers, auth=('devr', 'DevReg\!robXtask')) 
            
        if r_get.ok and r_add.ok:
            print("------------------------------------")
            print("Result of self registration:")
            print("Entry already existed and was updated")
            print("------------------------------------")
        elif r_add.ok:
            print("------------------------------------")
            print("Result of self registration:")
            print("Description uploaded succesfully")
            print("------------------------------------")
        else:
            print("------------------------------------")
            print("Result of self registration:")
            print("Error in server response: " + str(r_add.status_code))
            print("------------------------------------")
                        
    except requests.exceptions.RequestException as e:
        print (e)
    
    
    
