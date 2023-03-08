from kinematics import inverseKinematics
import sys
from numpy import rad2deg

if __name__ == "__main__":
    sol, phi = inverseKinematics(float(sys.argv[1]), float(sys.argv[2]), phi_lo=int(sys.argv[3]), phi_hi=int(sys.argv[4]))
    print(f"base: {rad2deg(sol[0])}, elbow: {rad2deg(sol[1])}, wrist: {rad2deg(sol[2])}, phi: {rad2deg(phi)}")