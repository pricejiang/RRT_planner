

class box():
    def __init__(self, Xc, epsilon):
        self.center = Xc
        self.epsilon = epsilon
        self.xg = Xc.state[0]
        self.yg = Xc.state[1]
        self.theta = Xc.state[2]
        self.left = self.xg-epsilon
        self.right = self.xg+epsilon
        self.top = self.yg-epsilon
        self.bottom = self.yg+epsilon
        self.lt = (self.xg-epsilon, self.yg-epsilon)
        self.rt = (self.xg+epsilon, self.yg-epsilon)
        self.lb = (self.xg-epsilon, self.yg+epsilon)
        self.rb = (self.xg+epsilon, self.yg+epsilon)

    def getCenter(self):
        return self.center

    def getSides(self):
        return self.lt, self.rt, self.lb, self.rb

def parseBox(boxes):
    theta_array = []
    for b in boxes:
        theta_array.append(b.theta)
    # Parse theta
    theta_array = sorted(theta_array)
    min_theta = theta_array[0]
    max_theta = theta_array[-1]

    Bk = boxes[-1]

    return Bk, (min_theta, max_theta)
