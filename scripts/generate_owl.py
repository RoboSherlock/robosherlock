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


NS = "{http://uima.apache.org/resourceSpecifier}"
INDIVIDUAL_PREFIX = "Instance"
INPUT_PROPERTY_NAME = "requiresInput"
OUTPUT_PROPERTY_NAME = "factResult"

ACTOR_INPUT_PROPERTY_NAME = "perceptualInputRequired"
ACTOR_OUTPUT_PROPERTY_NAME = "perceptualOutput"

ACCEPTABLE_PERCEPTION_CAPABILITY_NAMES = ["Perceive3DDepthCapability", "PerceiveColorCapability", "PerceiveThermalCapability"]

def convertTypeToOntologyFormat(s):
    """Converts something like iai_rs.pcl.PointCloud to RsPclPointcloud"""
    return string.capwords(s,'.').replace(".","").replace("_","").replace("Iairs","Rs")

class Annotator:
    def __init__(self, name, subclass_of):
        self.name = name
        self.subclass_of = subclass_of
        self.inputs = []
        self.outputs = []
        self.required_capabilities = []

    # calculate the name that should be written to the Ontology from the given name
    def ontology_name(self):
        return self.name

    # calculate the superclass that should be written to the Ontology from the given subclass_of attribute
    def ontology_subclass_of(self):
        return self.subclass_of[:1].upper()+self.subclass_of[1:]+"Component"

    def ontology_inputs(self):
        return map(convertTypeToOntologyFormat, self.inputs)
        
    def ontology_outputs(self):
        return map(convertTypeToOntologyFormat, self.outputs)

    def ontology_required_capabilities(self):
        return self.required_capabilities
        
# Contains information about the superclass of a Class
class OWLSubClassOf:
    # restrictions should be a list of tupels with the format (PropertyName, SomeValuesFrom)
    def __init__(self,superclass_name,restrictions=[]):
        self.superclass_name = superclass_name
        self.restrictions = restrictions

class OWLClass:
    # set in_namespace to true, if the software should prefix the name and the subclass_of with the given prefix
    # in the OWLManager
    def __init__(self, name, subclass_of, in_namespace=True):
        self.name = name
        self.subclass_of = subclass_of
        self.in_namespace = in_namespace

class OWLIndividual(object):
    def __init__(self,name,resource, relations=[]):
        self.name = name
        self.resource = resource
        self.relations = relations

class OWLNamedIndividual(OWLIndividual):
    def __init__(self, name,resource, relations=[]):
        super(OWLNamedIndividual, self).__init__(name,resource,relations)

class OWLProperty(object):
    def __init__(self,name,domain="",property_range="",subproperty_of=""):
        self.name = name
        self.domain = domain
        self.property_range = property_range
        self.subproperty_of = subproperty_of

class OWLObjectProperty(OWLProperty):
    def __init__(self,name,domain="",property_range="",subproperty_of=""):
        super(OWLObjectProperty, self).__init__(name,domain,property_range,subproperty_of)
        
        

class OWLWriteManager:
    """Class to collect all the owl entities that should be written. You can
    add instances of OWLClasses, OWLProperties and OWLIndividuals to this manager.
    If you set up all your desired OWL Individuals, call writeOWLFile to write everything
    to a OWL file on your filesystem."""
    def __init__(self, namespace_prefix,file_name):
        self.namespace_prefix = namespace_prefix
        self.file_name = file_name
        self.owl_classes = []
        self.owl_properties = []
        self.owl_individuals = []
        self.ontology_description = ""

    def addOWLClass(self,c):
        self.owl_classes.append(c)

    def addOWLProperty(self, prop):
        self.owl_properties.append(prop)

    def addOWLIndividual(self, i):
        self.owl_individuals.append(i)

    # Untested
    def addRelationToIndividual(self, name, relations):
        for i in self.owl_individuals:
            if i.name == name:
                i.relations.append(relations)

    def setOntologyDescription(self, desc):
        """Set the description of the Ontology. This will be put into the rdfs:comment section of the ontology definition inside the header"""
        self.ontology_description = desc

    def genOWLHeader(self):
        """Generate a string for the header of the file"""
        rstr = ("<?xml version=\"1.0\"?>\n"
            "<!DOCTYPE rdf:RDF [\n"
            "   <!ENTITY owl \"http://www.w3.org/2002/07/owl#\" >\n"
            "   <!ENTITY owl \"http://www.w3.org/2002/07/owl#\" >\n"
            "   <!ENTITY owl \"http://www.w3.org/2002/07/owl#\" >\n"
            "   <!ENTITY swrl \"http://www.w3.org/2003/11/swrl#\" >\n"
            "   <!ENTITY xsd \"http://www.w3.org/2001/XMLSchema#\" >\n"
            "   <!ENTITY knowrob \"http://knowrob.org/kb/knowrob.owl#\" >\n"
            "   <!ENTITY srdl2-cap \"http://knowrob.org/kb/srdl2-cap.owl#\" >\n"
            "   <!ENTITY srdl2-comp \"http://knowrob.org/kb/srdl2-comp.owl#\" >\n"
            "   <!ENTITY srdl2-action \"http://knowrob.org/kb/srdl2-action.owl#\" >\n"
            "   <!ENTITY computable \"http://knowrob.org/kb/computable.owl#\" >\n"
            "   <!ENTITY rdfs \"http://www.w3.org/2000/01/rdf-schema#\" >\n"
            "   <!ENTITY rdf \"http://www.w3.org/1999/02/22-rdf-syntax-ns#\" >\n"
            "]>\n\n")
        rstr+= ("<rdf:RDF xmlns=\"http://knowrob.org/kb/" + self.file_name + "#\"\n"
            "     xml:base=\"http://knowrob.org/kb/" + self.file_name + "\"\n"
            "     xmlns:rdfs=\"http://www.w3.org/2000/01/rdf-schema#\"\n"
            "     xmlns:computable=\"http://knowrob.org/kb/computable.owl#\"\n"
            "     xmlns:swrl=\"http://www.w3.org/2003/11/swrl#\"\n"
            "     xmlns:owl=\"http://www.w3.org/2002/07/owl#\"\n"
            "     xmlns:xsd=\"http://www.w3.org/2001/XMLSchema#\"\n"
            "     xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\"\n"
            "     xmlns:knowrob=\"http://knowrob.org/kb/knowrob.owl#\"\n"
            "     xmlns:srdl2-cap=\"http://knowrob.org/kb/srdl2-cap.owl#\"\n"
            "     xmlns:srdl2-comp=\"http://knowrob.org/kb/srdl2-comp.owl#\"\n"
            "     xmlns:srdl2-action=\"http://knowrob.org/kb/srdl2-action.owl#\">\n"
            "     <owl:Ontology rdf:about=\"http://knowrob.org/kb/" + self.file_name + "\">\n"
            "        <rdfs:comment rdf:datatype=\"&xsd;string\">Annotators that are used in "
            "the RoboSherlock Framework\n"
            "        </rdfs:comment>\n"
            "     <owl:imports rdf:resource=\"package://knowrob_common/owl/knowrob.owl\"/>\n"
            "     <owl:imports rdf:resource=\"package://knowrob_srdl/owl/srdl2-action.owl\"/>\n"
            "     <owl:imports rdf:resource=\"package://knowrob_srdl/owl/srdl2-cap.owl\"/>\n"
            "     <owl:imports rdf:resource=\"package://knowrob_srdl/owl/srdl2-comp.owl\"/>\n"
            "    </owl:Ontology>\n\n\n"
            "<!-- http://knowrob.org/kb/" + self.file_name + "#RoboSherlockComponent-->\n"
            "<owl:Class rdf:about=\"http://knowrob.org/kb/" + self.file_name + "#RoboSherlockComponent\">\n"
            "   <rdfs:subClassOf rdf:resource=\"&knowrob;Algorithm\"/>\n"
            "</owl:Class>\n\n"
            "<!-- http://knowrob.org/kb/" + self.file_name + "#RoboSherlockType-->\n"
            "<owl:Class rdf:about=\"http://knowrob.org/kb/" + self.file_name + "#RoboSherlockType\">\n"
            "   <rdfs:subClassOf rdf:resource=\"&knowrob;ObjectType\"/>\n"
            "</owl:Class>\n\n")

        return rstr

    def genCommentBlock(self, comment):
        """Generate a string with a big block comment that's easily visible in the OWL file """
        rstr = ("<!--\n"
            "////////////////////////////////////////////////////\n"
            "//\n"
            "// "+comment+"\n"
            "//\n"
            "///////////////////////////////////////////////////\n"
            "-->\n\n\n")
        return rstr

    def genClasses(self):
        """Generate a string for all collected classes"""
        string=""
        for c in self.owl_classes:
            if c.in_namespace:
                full_class_name = self.namespace_prefix + "#" + c.name
                full_subclass_of = self.namespace_prefix + "#" + c.subclass_of.superclass_name
            else:
                full_class_name = c.name
                full_subclass_of = c.subclass_of.superclass_name
            
            if c.subclass_of.superclass_name:
                string += ("<!-- "+ full_class_name + " -->\n"
                        "<owl:Class rdf:about=\"" + full_class_name +"\">\n"
                        "    <rdfs:subClassOf rdf:resource=\"" + full_subclass_of + "\"/>\n")
            else:
                string += ("<!-- "+ full_class_name + " -->\n"
                        "<owl:Class rdf:about=\"" + full_class_name +"\">\n")

            for r in c.subclass_of.restrictions:
                (onProperty, someValuesFrom) = r
                # full_onproperty = self.namespace_prefix + "#" + onProperty
                # full_someValuesFrom = self.namespace_prefix + "#" + someValuesFrom
                full_onproperty = onProperty
                # if someValuesFrom is a list,
                # we a have complex someValuesFrom directive, which contains an anonymous class
                if isinstance(someValuesFrom,list):
                    # full_someValuesFrom = someValuesFrom
                    if len(someValuesFrom) < 3:
                        print "provided someValuesFrom with size < 3. It MUST consist of the type of the value definition (intersectionOf, unionOf, complementOf)"
                        print "plus atleast two class names"
                        exit(0)

                    typeOfValue = someValuesFrom[0]
                    string += (
                        "    <rdfs:subClassOf>\n"
                        "        <owl:Restriction>\n"
                        "            <owl:onProperty rdf:resource=\"" + full_onproperty + "\"/>\n"
                        "            <owl:someValuesFrom>\n"
                        "                <owl:Class>\n"
                        "                    <owl:"+ typeOfValue +" rdf:parseType=\"Collection\">\n")
                    for cl in someValuesFrom[1:]:
                        if c.in_namespace:
                            string+= "                        <rdf:Description rdf:about=\"" + self.namespace_prefix + "#" + cl +"\"/>\n"
                        else:
                            string+= "                        <rdf:Description rdf:about=\""+ cl +"\"/>\n"

                    string +=(
                        "                    </owl:"+ typeOfValue +">\n"
                        "                </owl:Class>\n"
                        "            </owl:someValuesFrom>\n"
                        "        </owl:Restriction>\n"
                        "    </rdfs:subClassOf>\n")
                else:
                    full_someValuesFrom = someValuesFrom
                    string += (
                        "    <rdfs:subClassOf>\n"
                        "        <owl:Restriction>\n"
                        "            <owl:onProperty rdf:resource=\"" + full_onproperty + "\"/>\n"
                        "            <owl:someValuesFrom rdf:resource=\"" + full_someValuesFrom + "\"/>\n"
                        "        </owl:Restriction>\n"
                        "    </rdfs:subClassOf>\n")

            string += (
                "</owl:Class>\n\n") 

        return string

    def genProperties(self):
        """Generate a string for all collected properties"""
        string=""
        for p in self.owl_properties:
            full_prop_name = self.namespace_prefix + "#" + p.name
            full_domain_name = self.namespace_prefix + "#" + p.domain
            full_range_name = self.namespace_prefix + "#" + p.property_range
            string += ("<owl:ObjectProperty rdf:about=\""+full_prop_name+"\">\n")
            if p.subproperty_of:
                string += "    <rdfs:subPropertyOf rdf:resource=\"" + p.subproperty_of + "\"/>\n"

            if p.domain and p.property_range:
                string += (
                   "    <rdfs:domain>\n"
                   "        <owl:Restriction>\n"
                   "            <owl:onProperty rdf:resource=\""+full_prop_name+"\"/>\n"
                   "            <owl:someValuesFrom rdf:resource=\""+full_domain_name+"\"/>\n"
                   "        </owl:Restriction>\n"
                   "   </rdfs:domain>\n"
                   "   <rdfs:range>\n"
                   "        <owl:Restriction>\n"
                   "            <owl:onProperty rdf:resource=\""+full_prop_name+"\"/>\n"
                   "            <owl:someValuesFrom rdf:resource=\""+full_range_name+"\"/>\n"
                   "        </owl:Restriction>\n"
                   "   </rdfs:range>\n"
                   "</owl:ObjectProperty>\n\n")
            else:
                string += "</owl:ObjectProperty>\n\n";
        return string

    def genOWLRelationAsString(self,relation_name, resource_name):
        # example: 
        # <requiresResultOfAlgorithm rdf:resource="http://knowrob.org/kb/rs_components.owl#RSAImagePreprocessor"/> 

        # should be inside a <owl:NamedIndividual> or <owl:AnonymousIndividual>
        return "    <"+relation_name+" rdf:resource=\""+self.namespace_prefix+"#"+resource_name+"\"/>\n"

    def genIndividuals(self):
        """Generate a string for all collected individuals. It only supports NamedIndividuals at the moment."""
        string=""
        for i in self.owl_individuals:
            full_individual_name = self.namespace_prefix + "#" + i.name
            full_resource = self.namespace_prefix + "#" + i.resource
            string+=("<!-- " + full_individual_name + " -->\n\n"
                    "<owl:NamedIndividual rdf:about=\"" + full_individual_name + "\">\n"
                    "    <rdf:type rdf:resource=\"" + full_resource + "\"/>\n")
            # Add Relations, if there are any
            for (relation_name, resource_name) in i.relations:
                string+=(self.genOWLRelationAsString(relation_name, resource_name))

            string+=("</owl:NamedIndividual>\n\n")

        return string

    def genOWLFile(self):
        """Generate a string for the whole ontology"""
        string = ""
        string += self.genOWLHeader()
        string += self.genCommentBlock("Object Properties")
        string += self.genProperties()
        string += self.genCommentBlock("Classes")
        string += self.genClasses()
        # string += self.genCommentBlock("Individuals")
        string += self.genIndividuals()
    
        string += ("</rdf:RDF>"
            "\n\n<!-- Generated by generateOWLDescription from the RoboSherlock project " + time.strftime("%d/%m/%Y")
            + "-->")
        return string
        
    def writeOWLFile(self):
        """Write all collected information to the OWL file specified in the class constructor"""
        f = open(self.file_name, 'w')
        f.write(self.genOWLFile())
        f.close()

    def __str__(self):
        rstr = "--OWLWriteManager Instance\n"
        rstr += "Filename: " + self.file_name + "\n"
        rstr += "Ontology Namespace Prefix: " + self.namespace_prefix + "\n"
        rstr += "Ontology Description: " + self.ontology_description + "\n"
        rstr += "Collected Classes ("+ str(len(self.owl_classes))+"): "
        for x in self.owl_classes:
            rstr+=str(x)
            rstr+=","
        rstr+= "\n"
        rstr += "Collected Properties ("+ str(len(self.owl_properties))+"): "
        for x in self.owl_properties:
            rstr+=str(x)
            rstr+=","
        rstr+= "\n"
        rstr += "Collected Individuals ("+ str(len(self.owl_individuals))+"): "
        for x in self.owl_individuals:
            rstr+=str(x)
            rstr+=","
        rstr+= "\n"

        return rstr

# Get path of Type files from Typesystem
def getTSPaths():

    rospack = rospkg.RosPack()
    core_path = rospack.get_path('iai_rs_cpp')
    core_path += str('/descriptors/typesystem/types')
    
    ts_paths = []
            
    for filename in os.listdir(core_path):
        filepath = os.path.join(core_path, filename)
        (name,ext) = os.path.splitext(filename)
        if not os.path.isfile(filepath) or ext != ".xml":
            continue;
        ts_paths.append(filepath) 
    return ts_paths


def getpackagepaths():

    rospack = rospkg.RosPack()
    paths = []
    paths.append(rospack.get_path('iai_rs_cpp'))
    package_names = rospack.get_depends_on('iai_rs_cpp')
    print('Packages depending on iai_rs_cpp:', package_names)
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
        if element.tag == NS+'sofaName':
            # print "sofaName: "+ element.text
            inputs.append(element.text)
        if element.tag == NS+'capability':
            t = {}
            for e in element:
                if e.tag == NS + 'outputs':
                    for types in e.findall(NS+'type'):
                        # print "Output Type: " + types.text
                        outputs.append(types.text)
    return (inputs,outputs)

def getAnnotatorRequirements(filepath):
    """Fetch the requirements of a Annotator from the annotator definition. Input: full filepath for a annotator
    description XML. It looks for "Requires CAPABILITYNAME" or "RequiresCAPABILITYNAME' in the 'description'
    field of the annotator XML. The usable Capabilitynames are defined at the heading of this file
    and should comply with the Capability Names in the rs_components.owl (knowrob_robosherlock package)
    below the PerceptionCapability class.
    Returns: A list of required Capabilities"""
    outputs = []
    inputs = []
    tree = etree.parse(filepath)
    root = tree.getroot()
    required_capabilities = []

    for element in root.iter("*"):
        if element.tag == NS+'description' and element.text:
            # print "description: "+ element.text
            for cap in ACCEPTABLE_PERCEPTION_CAPABILITY_NAMES:
                if "Requires"+cap in element.text or "Requires "+cap in element.text:
                    required_capabilities.append(cap)

    return required_capabilities

def getAnnotatorNames():

    paths = getpackagepaths()
    # print paths
    annotators = []
    atypes = []
            
    for p in paths:
        for subdir in os.walk(p).next()[1]:
            atypes.append(subdir)
            subdirpath = os.path.join(p,subdir)         
            for filename in os.listdir(subdirpath):
                 filepath = os.path.join(subdirpath, filename)
                 (name,ext) = os.path.splitext(filename)
                 if not os.path.isfile(filepath) or ext != ".xml":
                     continue;
                 
                 annotators.append(Annotator(name, subdir))
                 # annotators.append((subdir,name))
                 # print "Annotator:" + filepath
                 (inputs,outputs) = getAnnotatorIOs(filepath)
                 annotators[-1].required_capabilities = getAnnotatorRequirements(filepath)
                 annotators[-1].inputs = inputs
                 annotators[-1].outputs = outputs
        for filename in os.listdir(p):
            filepath = os.path.join(p, filename)
            (name,ext) = os.path.splitext(filename)
            if not os.path.isfile(filepath) or ext != ".xml":
                continue;
            # annotators.append(('RoboSherlock',name)) 
            annotators.append(Annotator(name, 'RoboSherlock'))
            # print "Annotator:" + filepath
            # getAnnotatorIOs(filepath)
            (inputs,outputs) = getAnnotatorIOs(filepath)
            annotators[-1].required_capabilities = getAnnotatorRequirements(filepath)
            annotators[-1].inputs = inputs
            annotators[-1].outputs = outputs
    return (atypes,annotators)
       
def getRoboSherlockTypes():
    """Read the required information from the RoboSherlock Typesystem"""
    result_list = []
    ts_paths = getTSPaths()    
    types = []
    for ts in ts_paths:
        (name,ext) = os.path.splitext(ntpath.basename(ts))

        tree = etree.parse(ts)
        root = tree.getroot()
        for element in root.iter("*"):
            if element.tag == NS+'typeDescription':
               t = {}
               for e in element:
                    if e.tag == NS +'name':
                        t['type'] = e.text
                    if e.tag == NS +'supertypeName':
                        t['supertype'] =e.text
               types.append(t)  
    
    for t in types:
        subclassof = convertTypeToOntologyFormat(t['supertype'])
        classname =  convertTypeToOntologyFormat(t['type'])
        if subclassof == "UimaCasTop":
            subclassof = "RoboSherlockType"
        result_list.append( (classname, subclassof) )

    return result_list

if __name__ == "__main__":
    print("Generating OWL description from UIMA descriptions")
    owl_manager = OWLWriteManager("http://knowrob.org/kb/rs_components.owl","rs_components.owl")

    # DEPRECATED: Define the rsInput and rsOutput relation. A RoboSherlockComponent can require a RoboSherlockType or yield one as a result.
    owl_manager.addOWLProperty( OWLObjectProperty(  INPUT_PROPERTY_NAME,"RoboSherlockComponent","RoboSherlockType") )
    owl_manager.addOWLProperty( OWLObjectProperty( OUTPUT_PROPERTY_NAME,"RoboSherlockComponent","RoboSherlockType") )
    
    # Define input/output property in KnowRob Ontology
    owl_manager.addOWLProperty( OWLObjectProperty( ACTOR_INPUT_PROPERTY_NAME, "RoboSherlockComponent","RoboSherlockType", "&knowrob;preActors") )
    owl_manager.addOWLProperty( OWLObjectProperty( ACTOR_OUTPUT_PROPERTY_NAME,"RoboSherlockComponent","RoboSherlockType", "&knowrob;outputs") )

    # Add all the RoboSherlock Types to the ontology
    list_of_types = getRoboSherlockTypes()
    for (classname,subclassof) in list_of_types:
        print classname+" is subclass of "+subclassof
        owl_manager.addOWLClass(OWLClass(classname, OWLSubClassOf(subclassof) ))
        # Add an individual for every type
        owl_manager.addOWLIndividual(OWLNamedIndividual( INDIVIDUAL_PREFIX + classname, classname)) 

    # Fetch the information about the annotators
    (atypes,annotators) = getAnnotatorNames()
    
    # Add main classes. This are the higher level classes like "io", "annotation", "segmentation", etc.
    # print atypes
    for folder in atypes:
        owl_manager.addOWLClass(OWLClass(folder[:1].upper()+folder[1:]+"Component",OWLSubClassOf("RoboSherlockComponent") ))
       
    # Add the actual Annotators
    for a in annotators:     
        print ("A Annotator: " + a.ontology_name() + " AND " + a.ontology_subclass_of() + 
                " |I|=" + str(len( a.ontology_inputs() )) + 
                " |O|=" + str(len( a.ontology_outputs() ))          
                )
        # Skip ROSBagBridge atm to avoid failures in planning
        if a.ontology_name() == "ROSBagBridge":
            continue
        # Collect input/output relations
        relations = []
        for ins in a.ontology_inputs() :
            print " Input: " + ins
            relations.append( ( INPUT_PROPERTY_NAME , ins ) )
        for outs in a.ontology_outputs() :
            print " Output: " + outs
            relations.append( ( OUTPUT_PROPERTY_NAME , outs ) )
        for cap in a.ontology_required_capabilities() :
            print " Required Capabilities: " + cap


        class_relations = []
        for ins in a.ontology_inputs() :
            # print " Input: " + ins
            class_relations.append( ( "http://knowrob.org/kb/rs_components.owl#"+ACTOR_INPUT_PROPERTY_NAME , ("http://knowrob.org/kb/rs_components.owl#"+ins) ) )
        for outs in a.ontology_outputs() :
            # print " Output: " + outs
            class_relations.append( ( "http://knowrob.org/kb/rs_components.owl#"+ACTOR_OUTPUT_PROPERTY_NAME , ("http://knowrob.org/kb/rs_components.owl#"+outs) ) )
        for cap in a.ontology_required_capabilities() :
            class_relations.append( ( "&srdl2-cap;dependsOnCapability" , "http://knowrob.org/kb/rs_components.owl#" + cap ) )

        # Add class and individuals
        owl_manager.addOWLClass(OWLClass(a.ontology_name(), OWLSubClassOf(a.ontology_subclass_of(),class_relations ) ) )
        owl_manager.addOWLIndividual(OWLNamedIndividual( INDIVIDUAL_PREFIX + a.ontology_name(), a.ontology_name(), relations ) ) 
        # owl_manageraddRelationToIndividual(self, name, relations)

     # Add Capabilites for Perception Planning
    relations = []
    # Create Perceive3DDepthCapability
    relations.append( ( "&srdl2-comp;dependsOnComponent" , 
        ["unionOf",
            "http://knowrob.org/kb/srdl2-comp.owl#ThreeDimensionalLaserScanner",
            "http://knowrob.org/kb/srdl2-comp.owl#TimeOfFlightCamera",
            "http://knowrob.org/kb/rs_components.owl#DepthCamera",
            ] ) )

    owl_manager.addOWLClass(OWLClass("http://knowrob.org/kb/rs_components.owl#Perceive3DDepthCapability" , OWLSubClassOf("http://knowrob.org/kb/srdl2-cap.owl#PerceptionCapability", relations), False ) )

    # Create PerceiveColorCapability
    relations = []
    relations.append( ( "&srdl2-comp;dependsOnComponent" , "http://knowrob.org/kb/srdl2-comp.owl#ColorCamera" ) )
    owl_manager.addOWLClass(OWLClass("http://knowrob.org/kb/rs_components.owl#PerceiveColorCapability" , OWLSubClassOf("http://knowrob.org/kb/srdl2-cap.owl#PerceptionCapability", relations), False ) )

    # Create PerceiveThermalCapability
    relations = []
    relations.append( ( "&srdl2-comp;dependsOnComponent" , "http://knowrob.org/kb/srdl2-comp.owl#InfraredCamera" ) )
    owl_manager.addOWLClass(OWLClass("http://knowrob.org/kb/rs_components.owl#PerceiveThermalCapability" , OWLSubClassOf("http://knowrob.org/kb/srdl2-cap.owl#PerceptionCapability", relations), False ) )

    # Create the Depth receiving part of the Kinect as Camera and assert it under the knowrob_srdl ontology
    owl_manager.addOWLClass(OWLClass("http://knowrob.org/kb/rs_components.owl#DepthCamera", OWLSubClassOf("http://knowrob.org/kb/srdl2-comp.owl#Camera") , False ))
    owl_manager.addOWLClass(OWLClass("http://knowrob.org/kb/rs_components.owl#KinectIRProjectionCamera", OWLSubClassOf("http://knowrob.org/kb/rs_components.owl#DepthCamera"),False ))

    # Create the hasVisualProperty object property to
    # describe the visual properties of an object
    owl_manager.addOWLProperty( OWLObjectProperty( "hasVisualProperty") )
    # and the hasDetectionClue object property
    # to provide background knowledge about perceptual things
    owl_manager.addOWLProperty( OWLObjectProperty( "hasDetectionClue") )

    # Create the visual appearances
    owl_manager.addOWLClass(OWLClass("VisualAppearance", OWLSubClassOf("") ))
    # The meta types of visual appearances:
    owl_manager.addOWLClass(OWLClass("Size", OWLSubClassOf("VisualAppearance") ))
    owl_manager.addOWLClass(OWLClass("Logo", OWLSubClassOf("VisualAppearance") ))
    owl_manager.addOWLClass(OWLClass("TextOnObject", OWLSubClassOf("VisualAppearance") ))
    owl_manager.addOWLClass(OWLClass("Shape", OWLSubClassOf("VisualAppearance") ))
    owl_manager.addOWLClass(OWLClass("HandlingPoints", OWLSubClassOf("VisualAppearance") ))

    owl_manager.addOWLClass(OWLClass("SmallSize", OWLSubClassOf("Size") ))
    owl_manager.addOWLClass(OWLClass("MediumSize", OWLSubClassOf("Size") ))
    owl_manager.addOWLClass(OWLClass("BigSize", OWLSubClassOf("Size") ))

    # owl_manager.addOWLClass(OWLClass("KellogsLogo", OWLSubClassOf("Logo") ))
    # owl_manager.addOWLClass(OWLClass("MondaminLogo", OWLSubClassOf("Logo") ))
    # owl_manager.addOWLClass(OWLClass("PfannerLogo", OWLSubClassOf("Logo") ))

    # owl_manager.addOWLClass(OWLClass("TextCornFlakes", OWLSubClassOf("TextOnObject") ))
    # owl_manager.addOWLClass(OWLClass("TextIceTea", OWLSubClassOf("TextOnObject") ))
    # owl_manager.addOWLClass(OWLClass("TextMilch", OWLSubClassOf("TextOnObject") ))
    # owl_manager.addOWLClass(OWLClass("TextPfannkuchenmix", OWLSubClassOf("TextOnObject") ))
    # owl_manager.addOWLClass(OWLClass("TextTee", OWLSubClassOf("TextOnObject") ))

    owl_manager.addOWLClass(OWLClass("BoxShape", OWLSubClassOf("Shape") ))
    owl_manager.addOWLClass(OWLClass("CylinderShape", OWLSubClassOf("Shape") ))
    owl_manager.addOWLClass(OWLClass("CylindricalShape", OWLSubClassOf("Shape") ))
    owl_manager.addOWLClass(OWLClass("SphereShape", OWLSubClassOf("Shape") ))
    owl_manager.addOWLClass(OWLClass("ConeShape", OWLSubClassOf("Shape") ))
    owl_manager.addOWLClass(OWLClass("RoundShape", OWLSubClassOf("Shape") ))

    owl_manager.addOWLClass(OWLClass("GraspPoints", OWLSubClassOf("HandlingPoints") ))
    owl_manager.addOWLClass(OWLClass("OpeningPoints", OWLSubClassOf("HandlingPoints") ))
    owl_manager.addOWLClass(OWLClass("Knob", OWLSubClassOf("GraspPoints") ))
    owl_manager.addOWLClass(OWLClass("OneHandGraspable", OWLSubClassOf("GraspPoints") ))
    owl_manager.addOWLClass(OWLClass("TwoHandGraspable", OWLSubClassOf("GraspPoints") ))

    # The second type of detection predicates: DetectionClues.
    # This can be things that are background knowledge about an object, for example
    # types of different models. 
    owl_manager.addOWLClass(OWLClass("DetectionClue", OWLSubClassOf("") ))
    owl_manager.addOWLClass(OWLClass("ModelDetection", OWLSubClassOf("DetectionClue") ))
    owl_manager.addOWLClass(OWLClass("BlortModel", OWLSubClassOf("ModelDetection") ))
    owl_manager.addOWLClass(OWLClass("LinemodModel", OWLSubClassOf("ModelDetection") ))
    owl_manager.addOWLClass(OWLClass("PancakeDetector", OWLSubClassOf("ModelDetection") ))

# 
#     <!-- http://knowrob.org/kb/rs_test_objects.owl#InDrawer -->
# 
#     <owl:Class rdf:about="http://knowrob.org/kb/rs_test_objects.owl#InDrawer">
#         <rdfs:subClassOf rdf:resource="http://knowrob.org/kb/rs_test_objects.owl#QualitativeLocationDescription"/>
#     </owl:Class>
#     
# 
# 
#     <!-- http://knowrob.org/kb/rs_test_objects.owl#InRefrigerator -->
# 
#     <owl:Class rdf:about="http://knowrob.org/kb/rs_test_objects.owl#InRefrigerator">
#         <rdfs:subClassOf rdf:resource="http://knowrob.org/kb/rs_test_objects.owl#QualitativeLocationDescription"/>
#     </owl:Class>
#     
# 
# 
#     <!-- http://knowrob.org/kb/rs_test_objects.owl#OnTable -->
# 
#     <owl:Class rdf:about="http://knowrob.org/kb/rs_test_objects.owl#OnTable">
#         <rdfs:subClassOf rdf:resource="http://knowrob.org/kb/rs_test_objects.owl#QualitativeLocationDescription"/>
#     </owl:Class>
    # <!-- http://knowrob.org/kb/rs_test_objects.owl#QualitativeLocationDescription -->

    # <owl:Class rdf:about="http://knowrob.org/kb/rs_test_objects.owl#QualitativeLocationDescription"/>
    

    print "Collected the following entities for the Ontology:"
    print str(len(owl_manager.owl_classes)) + " Classes"
    print str(len(owl_manager.owl_properties)) + " Properties"
    print str(len(owl_manager.owl_individuals)) + " Individuals"

    owl_manager.writeOWLFile()
    print 'Done'
