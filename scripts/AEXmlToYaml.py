#!/usr/bin/env python
# -*- coding: utf-8 -*-


# TODO: use variable for rs_components namespace

import os
import rospkg
import time
import string
import ntpath

from os import listdir
from os.path import isfile, join
from lxml import etree
import yaml
NS = "{http://uima.apache.org/resourceSpecifier}"



class Annotator:
    def __init__(self):
        self.annotator = dict()
        self.parameters = dict()
        self.capabilities = dict()
        self.name = ""

    def writeToYamlFile(self, filepath):
        yamldata = dict()
        (pathTrunk, ext) = os.path.splitext(filepath)
        yamldata['annotator']= self.annotator
        yamldata['parameters'] = self.parameters
        yamldata['capabilities'] = self.capabilities
        print self.name+ ':', yamldata
        with open(pathTrunk +".yaml", 'w') as outfile:
            yaml.dump(yamldata, outfile, default_flow_style=False)

    def fromXML(self, xmlPath):

        (p, ext) = os.path.splitext(xmlPath)
        if ext != ".xml":
            return False

        (inputs, outputs) = getAnnotatorIOs(xmlPath)

        self.capabilities['inputs'] = inputs
        self.capabilities['outputs'] = outputs

        tree = etree.parse(xmlPath)
        root = tree.getroot()

        # aeMetaData = etree.SubElement(root, NS + 'analysisEngineMetaData')
        #
        # for element in aeMetaData.iter:
        #     print element
        for elem in root.iter("*"):
            if elem.tag == NS + 'annotatorImplementationName':
                self.annotator["implementation"] = elem.text
            if elem.tag == NS + 'analysisEngineMetaData':
                for e in elem:
                    if e.tag == NS + 'name':
                        self.annotator['name'] = e.text
                        self.name = elem.text
                    if e.tag == NS + 'description':
                        if e.text!=None:
                            self.annotator['description'] = e.text
            if elem.tag == NS +'configurationParameterSettings':
                for paramdefs in elem:
                    pname = ''
                    value= ''
                    array_value = []
                    array_type = ''
                    type = ''
                    for p in paramdefs:
                        if p.tag == NS + 'name':
                            pname = p.text
                        if p.tag == NS+ 'value':
                            for val in p:
                                if val.tag == NS+'string':
                                    type = 'string'
                                    value = val.text
                                if val.tag == NS + 'integer':
                                    value = val.text
                                    type = 'integer'
                                if val.tag == NS +'boolean':
                                    value = val.text
                                    type = 'boolean'
                                if val.tag == NS + 'float':
                                    value = val.text
                                    type = 'float'
                                if val.tag == NS+ 'array':
                                    for v in val:
                                        array_type = v.tag[len(NS):]
                                        if array_type == 'integer':
                                            array_value.append(int(v.text))
                                        elif array_type == 'float':
                                            array_value.append(float(v.text))
                                        elif array_type == 'boolean':
                                            array_value.append(bool(v.text))
                                        else:
                                            array_value.append(v.text)

                    if value !='':
                        if type == 'integer':
                            self.parameters[pname] = int(value)
                        elif type == 'boolean':
                            self.parameters[pname] = bool(value)
                        elif type == 'float':
                            if(value.endswith('f')):
                                self.parameters[pname] = float(value[:-1])
                            else:
                                self.parameters[pname] = float(value)
                        else:
                            self.parameters[pname] = value
                    else:
                        self.parameters[pname] = array_value
        return True


def getpackagepaths():
    rospack = rospkg.RosPack()
    paths = []
    paths.append(rospack.get_path('robosherlock'))
    package_names = rospack.get_depends_on('robosherlock')
    print('Packages depending on RoboSherlock:', package_names)
    for pn in package_names:
        paths.append(rospack.get_path(pn))

    for i in range(len(paths)):
        paths[i] += str('/descriptors/annotators')
    return paths


def getAnnotatorIOs(filepath):
    """Fetch the I/O from the annotator definition. Input: full filepath for a annotator
    description XML.
    Returns: A tuple of that form: (list-of-inputs, list-of-outputs)"""
    outputs = []
    inputs = []
    tree = etree.parse(filepath)
    root = tree.getroot()
    for element in root.iter("*"):
        if element.tag == NS + 'sofaName':
            # print "sofaName: "+ element.text
            inputs.append(element.text)
        if element.tag == NS + 'capability':
            t = {}
            for e in element:
                if e.tag == NS + 'outputs':
                    for types in e.findall(NS + 'type'):
                        # print "Output Type: " + types.text
                        outputs.append(types.text)
    return (inputs, outputs)

def getAnnotatorInfo(package_path):
    if not os.path.isdir(package_path):
        return
    for subdir in os.walk(package_path).next()[1]:
        subdirpath = os.path.join(package_path, subdir)
        getAnnotatorInfo(subdirpath)
    for filename in os.listdir(package_path):
        filepath = os.path.join(package_path, filename)
        if not os.path.isfile(filepath):
            continue
        annotator = Annotator()
        if annotator.fromXML(filepath):
            annotator.writeToYamlFile(filepath)


if __name__ == "__main__":

    annotators = []

    paths = getpackagepaths()
    for p in paths:
        getAnnotatorInfo(p)


