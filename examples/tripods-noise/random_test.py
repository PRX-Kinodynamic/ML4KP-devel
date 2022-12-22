import NoisyTimeMap
import ctypes
TM = NoisyTimeMap.NoisyTimeMap("examples/tripods/compute_roa.yaml")

start_state=[-3.02159,-6.28318]
print("Using default seed")
print("x_T", TM.g_func(start_state))
print("x_T'", TM.g_func(start_state))
print("x_T''", TM.g_func(start_state))


print("Fixing the seed")
TM.set_seed(112392);
print("x_T", TM.g_func(start_state))
TM.set_seed(112392);
print("x_T", TM.g_func(start_state))
TM.set_seed(112392);
print("x_T", TM.g_func(start_state))

def g_fix_seed(X):
	TM.set_seed(ctypes.c_ulonglong(hash(tuple(X))).value);
	return TM.g_func(X)

print("Fixing the seed based on the start state")
print("x_T", g_fix_seed(start_state))
print("x_T", g_fix_seed(start_state))

print("Different state")

other_start_state=[0,0]
other_start_state[0] = start_state[0] +1
other_start_state[1] = start_state[1]

print("Other x_T", g_fix_seed(other_start_state))
print("Other x_T", g_fix_seed(other_start_state))

print("Original x_T", g_fix_seed(start_state))
print("Other x_T", g_fix_seed(other_start_state))