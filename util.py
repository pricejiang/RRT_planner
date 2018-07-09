import importlib
import sys
import pdb

def parseInput(arg):
    model = arg[-1]
    path = "model/"+model
    # print path
    return import_newState(path)
    

def import_newState(path):
    path = path.replace('/','.')
    # pdb.set_trace()
    try:
        module = importlib.import_module(path)
    except:
        print("Import simulation function failed!")
    return module.selectInput, module.randomConfig, module.tryInput
    

if __name__ == '__main__':
    parseInput(sys.argv)
