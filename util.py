import importlib
import sys
import pdb
import json

def parseInput(arg):
    assert ".json" in sys.argv[-1], "Please provide json input file"
    with open(sys.argv[-1], 'r') as f:
        data = json.load(f)
    # model = arg[-1]
    # path = "model/"+model
    # print path
        module = import_newState(data["directory"])
        return module, data
    

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
