# Simple battery sizing for humanoid robots (illustrative)
def battery_size(P_avg, P_peak, t_hours, V_nom, eta_sys,
                 cell_capacity_Ah, cell_Crate, R_int_per_pack):
    # required pack capacity in Ah (eq. capacity)
    C_req = (P_avg * t_hours) / (V_nom * eta_sys)
    # peak current (eq. peak_current)
    I_peak = P_peak / V_nom
    # implied C-rate for pack
    implied_C = I_peak / C_req if C_req>0 else float('inf')
    # voltage sag under peak
    V_term = V_nom - I_peak * R_int_per_pack
    # available cell count estimate (in series to reach V_nom)
    cells_in_series = max(1, int(round(V_nom / 3.7)))  # typical Li-ion nominal 3.7V
    # pack Ah if using one parallel string of cells
    pack_Ah = cell_capacity_Ah
    # safety checks
    can_handle_peak = (cell_Crate * pack_Ah) >= I_peak
    return dict(C_required_Ah=C_req, I_peak_A=I_peak,
                implied_C=implied_C, V_term_peak=V_term,
                cells_series=cells_in_series, can_handle_peak=can_handle_peak)

# Example usage with realistic numbers
params = battery_size(P_avg=200, P_peak=1500, t_hours=2, V_nom=48,
                      eta_sys=0.9, cell_capacity_Ah=3.0, cell_Crate=10,
                      R_int_per_pack=0.05)  # 50 mOhm pack R_int
print(params)  # inline comment: use results to inform pack architecture