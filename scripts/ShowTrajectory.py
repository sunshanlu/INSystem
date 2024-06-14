import matplotlib.pyplot as plt
import pandas as pd
from sys import argv

def ShowTrajectory(TraFlie: str):
    data = pd.read_csv(TraFlie, sep=" ")
    plt.plot(data["x"], data["y"], "r-")
    plt.show()

if __name__ == "__main__":
    ShowTrajectory(argv[1])

