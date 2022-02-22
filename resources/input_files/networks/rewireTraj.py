
## does a binary search
def find_maximal_state_old(s, traj):
    lower, upper = 0, len(traj)-1
    idx = -1
    opt_traj = []
    
    flag, cand_traj = reachable(s, traj[-1])
    if flag: return len(traj)-1, cand_traj
    
    while lower <= upper:
        mid = int((lower + upper)/2)
        flag, cand_traj = reachable(s, traj[mid])
        if flag:
            opt_traj = cand_traj
            lower = mid+1
            idx = mid
        else:
            upper = mid-1
    return idx, opt_traj

## finds maximal state iteratively 
def find_maximal_state_new(s, traj):
    for idx in range(len(traj)):
        success, opt_traj = reachable(s, traj[idx])
        if success:
            return idx, opt_traj
    return -1, []

def rewire_traj(traj):
    rewired_traj, current_states, local_goals, last_states, state_num = [], [], [], [], 0
    
    while state_num < (len(traj)-1):
        max_idx, opt_traj = find_maximal_state(traj[state_num], traj[state_num+1:])
        if max_idx == -1: 
            print('Rewiring Has Failed')
            max_idx = state_num + 1
            opt_traj = [traj[state_num]]
        else: 
            max_idx += state_num+1
            for s in opt_traj:
                current_states.append(s)
                local_goals.append(traj[max_idx])
                last_states.append(opt_traj[-1])
            traj[max_idx] = opt_traj[-1]
        state_num = max_idx
        rewired_traj += opt_traj 
        
    return rewired_traj, current_states, local_goals, last_states