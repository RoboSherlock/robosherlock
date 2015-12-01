# -*- coding: utf-8 -*-
"""
Created on Wed Nov 11 15:28:59 2015

@author: ferenc
"""

import httplib


class GogglesClient:

    
    def __init__(self):

        self.conn = httplib.HTTPConnection('www.google.com')
        self.headers = {"User-Agent": "Mozilla/5.0 (Linux; U; Android 2.3.4; de-de; GT-I9000 Build/GINGERBREAD) AppleWebKit/533.1 (KHTML, like Gecko) Version/4.0 Mobile Safari/533.1 GoogleGoggles-Android/1.7; gzip",
                   "Content-Type": "application/x-protobuffer",
                   "Pragma": "no-cache",
                   "Connection": "close"}
        self.url = "/goggles/a/singleshot_search_proto"

    def call_service(self,body):
        print "Calling http service"
        self.conn.request("POST",self.url,body,self.headers)
        response = self.conn.getresponse();
        print response.status
        return response.read()
    def dummyPrint(self):
        print "Dummy function called"    


if __name__ == "__main__":
    client = GogglesClient()
    f = open('/home/ferenc/serializedProtoMessage.txt', 'rb')
    message = f.read()
    client.call_service(message)

#resp = conn.getresponse()
#print resp.status
#print resp.read()
