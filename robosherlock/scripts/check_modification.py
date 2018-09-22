import sys, os
import pickle
import hashlib

################################################################################
# check_modification
################################################################################

def isModified(files):
  ext = hashlib.md5("".join(sys.argv)).hexdigest()
  (path, script) = os.path.split(sys.argv[0])
  modFilename = ".{}_{}.modified".format(os.path.splitext(script)[0], ext)
  if os.path.exists(modFilename) and os.path.isfile(modFilename):
    try:
      modOld = pickle.load(open(modFilename, "rb"))
    except:
      print("Failed to read from '" + modFilename + "'.")
      modOld = {}
  else:
    modOld = {}
  modNew = {}
  modified = False

  fs = os.stat(sys.argv[0])
  if not sys.argv[0] in modOld or modOld[sys.argv[0]] != fs.st_mtime:
    modified = True
  modNew[sys.argv[0]] = fs.st_mtime

  for filepath in files:
    fs = os.stat(filepath)
    if not filepath in modOld or modOld[filepath] != fs.st_mtime:
      modified = True
    modNew[filepath] = fs.st_mtime

  for key in modOld.iterkeys():
    modified = modified or not key in modNew

  if modified:
    try:
      pickle.dump(modNew, open(modFilename, "wb"))
    except:
      print("Failed to dump to '" + modFilename + "'.")
  return modified

def removeModified():
  ext = hashlib.md5("".join(sys.argv)).hexdigest()
  (path, script) = os.path.split(sys.argv[0])
  modFilename = ".{}_{}.modified".format(os.path.splitext(script)[0], ext)
  if os.path.exists(modFilename):
    os.remove(modFilename)
  return
