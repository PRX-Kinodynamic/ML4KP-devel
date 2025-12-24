import gtsam

# X -> pose of the object; G -> Goal pose

# The direction of movement for an SE(2) from X to G is:
# \tau = LogMap(X^-1 * G) => in X local frame
# \tau = LogMap(G * X^-1) => in Global frame
# The inverse is G = X * ExpMap(tau)

# X == G
x = gtsam.Pose2(0,1,2)
G = gtsam.Pose2(0,1,2)
tau0 = gtsam.Pose2.Logmap(x.inverse() * G)
Gp0 = x * gtsam.Pose2.Expmap(tau0)

print("X", x)
print("G", G)
print("Tau",tau0)
print("Gp", Gp0)
print("-~-~-~-~-~-~-~-~-~-~-~-~")

G = gtsam.Pose2(1,1,2)
tau1 = gtsam.Pose2.Logmap(x.inverse() * G)
tau1p = gtsam.Pose2.Logmap( G * x.inverse())
Gp1 = x * gtsam.Pose2.Expmap(tau1)
Gp1p = gtsam.Pose2.Expmap(tau1p) * x

print("X", x)
print("G", G)
print("Tau (in X local Frame)",tau1)
print("Tau (in Global Frame)", tau1p)
print("Gp", Gp1)
print("Gp'", Gp1p)
print("-~-~-~-~-~-~-~-~-~-~-~-~")

G = gtsam.Pose2(1, 3, -2)
tau2 = gtsam.Pose2.Logmap(x.inverse() * G)
tau2p = gtsam.Pose2.Logmap( G * x.inverse())
Gp2 = x * gtsam.Pose2.Expmap(tau2)
Gp2p = gtsam.Pose2.Expmap(tau2p) * x

print("X", x)
print("G", G)
print("Tau (in X local Frame)",tau2)
print("Tau (in Global Frame)", tau2p)
print("Gp", Gp2)
print("Gp'", Gp2p)