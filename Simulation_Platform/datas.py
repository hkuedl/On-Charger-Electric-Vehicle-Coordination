import pandas as pd
import numpy as np
import glob
import random
import torch
import torch.utils.data as data
device = "cuda" if torch.cuda.is_available() else "cpu"

def setup_seed(seed: int = 1234):
    """set a fix random seed.
    
    Args:
        seed (int, optional): random seed.
    """
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    np.random.seed(seed)
    random.seed(seed)
    torch.backends.cudnn.deterministic = True

def load_data(path, postfix, choose=None):
    files = sorted(glob.glob(path + postfix))
    if type(choose) is int:
        print(f"building: {files[choose]}")
        df = pd.read_csv(files[choose])
        return df
    elif type(choose) is str:
        file = glob.glob(path + choose + postfix)[0]
        df = pd.read_csv(file)
        return df
    elif type(choose) is list:
        dfs = []
        for file in choose:
            if type(file) is int:
                print(f"building: {files[file]}")
                dfs.append(pd.read_csv(files[file]))
            elif type(file) is str:
                file = glob.glob(path + file + postfix)[0]
                dfs.append(pd.read_csv(file))
        return dfs
    elif choose is None:
        random_choose = np.random.randint(0, len(files))
        df = pd.read_csv(files[random_choose])
        return df
    else:
        dfs = []
        for file in files:
            dfs.append(pd.read_csv(file))
        return dfs, files
