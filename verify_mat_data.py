import scipy.io as sio

data = sio.loadmat('results/tuning_experiments_data.mat')
lqr = data['lqr_results']
fixed = data['fixed_results']
all_res = data['all_results']
road_names = [r[0] for r in data['road_names'][0]]

def get_val(arr, field):
    v = arr[field].item()
    while hasattr(v, '__len__') and not isinstance(v, (str, bytes)):
        v = v[0]
    return float(v)

for r in range(6):
    r_name = road_names[r]
    print(f"\n=== ROAD {r+1}: {r_name} ===")
    
    # LQR
    l = lqr[r, 0]
    print(f"LQR:    SS={get_val(l, 'rms_acc_ss'):.4f}, Tot={get_val(l, 'rms_acc_tot'):.4f}, PkAcc={get_val(l, 'pk_acc'):.4f}, PkStroke={get_val(l, 'pk_def'):.2f} mm, PkForce={get_val(l, 'pk_u'):.2f} N")
    
    # Fixed
    f = fixed[r, 0]
    print(f"Fixed:  SS={get_val(f, 'rms_acc_ss'):.4f}, Tot={get_val(f, 'rms_acc_tot'):.4f}, PkAcc={get_val(f, 'pk_acc'):.4f}, PkStroke={get_val(f, 'pk_def'):.2f} mm, PkForce={get_val(f, 'pk_u'):.2f} N")
    
    # Cand 5
    c = all_res[r, 4]
    print(f"Cand 5: SS={get_val(c, 'rms_acc_ss'):.4f}, Tot={get_val(c, 'rms_acc_tot'):.4f}, PkAcc={get_val(c, 'pk_acc'):.4f}, PkStroke={get_val(c, 'pk_def'):.2f} mm, PkForce={get_val(c, 'pk_u'):.2f} N | (c1={get_val(c, 'c1'):.1f}, k1={get_val(c, 'k1'):.1f}, k2={get_val(c, 'k2'):.1f})")
