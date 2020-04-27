#!/usr/bin/python3
from pymoos import pymoos
import time
import threading
import re
import numpy as np
from numpy import genfromtxt, sqrt
import matplotlib.pyplot as plt
from math import pow, ceil, exp, pi
from numpy.ma import zeros
from numpy import genfromtxt, sqrt
# import range_only_functions_moos as rof
import csv


__author__ = "jim keane"
__date__ = "05 04 20"
__ver__ = "0.2"  # the slow translation prcess

class ownshipMOOS(pymoos.comms):
    """
    JR KEANE
    05 Apr 2020
    Holthouse homing code

    ownship is running onboard the auv to build the db of range reports for APCKF
    It registers for 'NODE_REPORTS', RANGE_USV_AUV and responds with waypoints of estimated
    usv position.

    Attributes:
        moos_community: a string representing the address of the Community
        moos_port:      an integer defining the port
        9000 is shoreside
        90001 is the auv server for MOOSDB

    """

    def __init__(self, moos_community, moos_port):
        """Initiates MOOSComms, sets the callbacks and runs the loop"""
        super(ownshipMOOS, self).__init__()
        self.server = moos_community
        self.port = moos_port
        self.name = 'ownshipMOOS'
        self.iter = 0

        self.usv_starting_quarter = 1 # 1 = NE, 2 = SE, 3 = SW, 4 = NW
        self.time_warp = 1 # out of action due to timesync issues at the mo

        self.show_circle = False
        self.new_range = False
        self.iter_acpkf = False
        # self.publish_marker = False
        self.csv_str = "ranges.csv"

        self.node = ""  # takes in whole node string
        self.range_report = ""  # coming in as range, time, auvx, auvy, usvx, usvy
        self.range_circle = ""
        self.range_pull = []
        self.range_time_moos = 0
        self.range_time_sim = 0
        self.sim_time = 0
        self.time_since_last_range = 0
        self.track_db = []
        self.db_r = []
        self.predictions_db = [[0,0]]
        self.r = 0
        self.r_x = 0
        self.r_y = 0

        self.n_x = 0
        self.n_y = 0

        self.lock = threading.Lock() # getting a lock to threadsafely draw

        self.add_active_queue('nav_queue', self.on_nav)
        self.add_message_route_to_active_queue('nav_queue', 'NAV_X')
        self.add_message_route_to_active_queue('nav_queue', 'NAV_Y')
        self.add_active_queue('range_queue', self.on_range)
        self.add_message_route_to_active_queue('range_queue', 'RANGE_USV_AUV')

        self.set_on_connect_callback(self.__on_connect)
        self.set_on_mail_callback(self.__on_new_mail)
        self.run(self.server, self.port, self.name)


    def __on_connect(self):
        """OnConnect callback"""
        print("Connected to", self.server, self.port,
              "under the name ", self.name)

        return (self.register('NODE_REPORT', 0)
                and self.register('NAV_X', 0)
                and self.register('NAV_Y', 0)
                and self.register('RANGE_USV_AUV', 0)
                and self.register('DB_UPTIME', 0))


    def __on_new_mail(self):
        """OnNewMail callback"""
        for msg in self.fetch():
            if msg.key() == 'NODE_REPORT':
                self.node = msg.string()
                # print("timewarp")
                # print(float(pymoos.get_moos_timewarp()))

            if msg.key() == 'DB_UPTIME':
                self.sim_time = msg.double()
                # print(self.sim_time)

        return True


    def on_range(self, msg):
        """Special callback for NAV_*"""
        # print("on_nav activated by",
        #       msg.key(), "with value", msg.double())
        if msg.key() == 'RANGE_USV_AUV':
            self.iter += 1
            self.range_report = msg.string()
            self.range_pull = re.findall(r"[-+]?\d*\.\d+|\d+", msg.string())
            self.r = float(self.range_pull[0])
            self.r_x = float(self.range_pull[2])
            self.r_y = float(self.range_pull[3])
            self.range_time_moos = msg.time()
            print(self.range_time_moos)
            # print(pymoos.time())

            if len(self.track_db) == 0:
                self.track_db = [[self.r, self.r_x, self.r_y, self.sim_time]]
                print("First")
            else:
                self.track_db.append([self.r, self.r_x, self.r_y, self.sim_time])
                # print("time since last * not accounting for time warp yet")
                last_time = self.track_db[self.iter-2][3]
                # print(self.range_time_moos-float(last_time))

            # print("moos time ")
            # print(self.range_time_moos)
            round_range = round(self.r)
            self.range_circle = "x="+str(self.r_x)+",y="+\
                                str(self.r_y)+",radius="+str(round_range)+",duration=60," \
                                "fill=0,label=range_"+str(self.iter)+",edge_color=yellow,fill_color=white,time=" \
                                + str(self.range_time_moos)+",edge_size=1"

            self.new_range = True
            self.show_circle = True

        return True


    def on_nav(self, msg):
        """Special callback for RANGE*"""
        # print("on_nav activated by",
        #       msg.key(), "with value", msg.double())

        if msg.key() == 'NAV_X':
            self.n_x = msg.double()
            self.notify('JIMX', float(self.n_x), -1)

        elif msg.key() == 'NAV_Y':
            self.n_y = msg.double()
            self.notify('JIMY', float(self.n_y), -1)

        return True


def apckf(db, N, usv_starting_quarter, time_warp):

    T = 15  # Sampling time in seconds (time between measurements)
    num_trk = 30  # Number of filters used in Angle-Parameterised Cubature Kalman filter (AP-CKF) # how to optimise
    sigma_r = 5  # standard deviation of range measurements (in meters)
    vmax = 0.5  # Maximum speed of assumed target in m/s
    vel_std = vmax / sqrt(3)  # This is the assumed standard deviation of target speed in m/s
    q_tild = 0  # This is the process noise intensity parameter in m^2/s^3.
    # MC = 1  # Number of Monte Carlo runs
    # rms = np.zeros(N)  # Root Mean Square position error is stored here
    rms = 0
    if usv_starting_quarter == 1:
        theta_min = np.radians(0)  # theta_min and theta_max specify the sector in which we expect the target
        theta_max = np.radians(90)

    elif usv_starting_quarter == 2:
        theta_min = np.radians(95)  # theta_min and theta_max specify the sector in which we expect the target
        theta_max = np.radians(180)

    elif usv_starting_quarter == 3:
        theta_min = np.radians(180)  # theta_min and theta_max specify the sector in which we expect the target
        theta_max = np.radians(271)

    else:
        theta_min = np.radians(-90)  # theta_min and theta_max specify the sector in which we expect the target
        theta_max = np.radians(0)

    if N == 1:
        print("estimate initial target pos in quadrant #" + str(usv_starting_quarter))
        print("theta min: "+str(theta_min)+ ", theta max: " + str(theta_max))

    # print("Initiliase APCKF where Q = covariance matrix & wt = weights of APCKF filters")
    R0 = db[0][0]  #  RO is first range used in initialisation
    # set up the width of angular region, given num_trk filters
    theta_width = (theta_max - theta_min) / num_trk
    # Now set up theta_vect for the mid-points (in the angular sectors) of all the filters
    theta_vect_min = (theta_min + theta_width / 2)
    theta_vect_max = (theta_max - theta_width / 2)
    theta_vect = np.arange(theta_vect_min, theta_vect_max, theta_width)
    # print(theta_vect)
    # init sigb0 as matrix of 1's the size of number of filters to be used
    sigb0 = np.ones(num_trk)
    sigb0 = sigb0 * float(theta_width / sqrt(12))  # the standard deviation in angle for each of the filters

    #  Set up Q, the process noise covariance matrix
    Q = np.array([[(1 / 3) * pow(T, 3), 0, (1 / 2) * pow(T, 2), 0],
                  [0, (1 / 3) * pow(T, 3), 0, (1 / 2) * pow(T, 2)],
                  [(1 / 2) * pow(T, 2), 0, T, 0],
                  [0, (1 / 2) * pow(T, 2), 0, T]])  # this matches Sanjeevs

    Q = q_tild * Q  # This is the process noise Covariance matrix
    wt = np.ones(num_trk)    # init wt array
    wt = wt * (1 / num_trk)  # fill with initial weights for the angle-parameterised filters
    x0 = np.zeros((4, num_trk))  # initializing the target states for each of the filters in the AP-CKF

    # initializing the target states for each of the filters in the AP-CKF
    sin_theta_vect = R0 * np.sin(theta_vect)
    cos_theta_vect = R0 * np.cos(theta_vect)
    x0[0, :] = sin_theta_vect + db[0][1]
    x0[1, :] = cos_theta_vect + db[0][2]

    # x0 initial target states for each of the filters. x0 shape and values"
    R_var = pow(sigma_r, 2)
    vel_var = pow(vel_std, 2)

    # The following 'for loop' initialises the covariance matrices for each of the num_trk filters
    first_P0 = True # treat first iter differently

    for i in range(num_trk): # where num_trk is Number of filters used in (AP-CKF)
        # print(i)
        sigma_bet = sigb0[i]
        bet = theta_vect[i]
        sigma_r2 = pow(sigma_r, 2)
        sigma_bet2 = pow(sigma_bet, 2)
        sin_bet2 = pow(np.sin(bet), 2)
        cos_bet2 = pow(np.cos(bet), 2)
        R02 = pow(R0, 2)
        sig_y2 = sigma_r2 * cos_bet2 + R02 * sigma_bet2 * sin_bet2
        sig_x2 = sigma_r2 * sin_bet2 + R02 * sigma_bet2 * cos_bet2
        sig_xy = ((sigma_r2) - (R02) * sigma_bet2) * np.sin(bet) * np.cos(bet)
        PP = np.array([[sig_x2, sig_xy],  # whatever PP stands for... these variable names are killing me
                       [sig_xy, sig_y2]])
        PP1 = np.zeros((2, 2))  # 2 x 2 of zeroes
        PP3 = np.eye(2) * vel_var  # where np.eye gives the identity matrix

        # P_stack = np.dstack((PP,PP1,PP1,PP3))  # this is how we can stack layers into 2x2xN
        top = np.hstack((PP, PP1))
        bottom = np.hstack((PP1, PP3))
        fourfour = np.vstack((top, bottom))  # stack the arrays into a 4 x 4. can speed this up later maybe
        if first_P0:
            P0 = np.copy(fourfour)
            first_P0 = False

        else:
            P0 = np.dstack((P0, fourfour))

    xf_all = np.copy(x0)
    P_all = np.copy(P0)
    xfm = np.zeros((4))  # xfm will contain the overall state estimate     # xfm translated okay.
    Pfiltm = np.zeros((4, 4))  # Pfiltm the corresponding covariance matrix
    # print("Initialised co-variance matrix PFILTM:")
    # print(Pfiltm)
    # print("Form the overall estimate of the AP-CKF based on the Gaussian-Sum formulation of the AP-CKF")
    for i in range(num_trk): # where num_trk is Number of filters used in (AP-CKF)
        # print(i+1)
        wti_x_xfa = wt[i] * xf_all[:, i]
        xfm = np.add(xfm, wti_x_xfa)  # need this xfm to iterate
        # Pfiltm_A = wt[i] * P_all[:, :, i]
        # print(Pfiltm_A)
        # Pfiltm_B = wt[i] * np.matmul(xf_col, xf_row)
        xf_row = np.array(xf_all[:, i], ndmin=2)
        xf_col = xf_row.reshape(-1, 1)
        Pfiltm = Pfiltm + wt[i] * P_all[:, :, i] + wt[i] * np.matmul(xf_col, xf_row)

    xf_transpose = xfm.reshape(-1, 1)
    Pfiltm = Pfiltm - xfm * xf_transpose
    xfs = np.empty((4, N))  # create estimated target states array
    Pfs = np.empty((4, 4, N))  # create corresponding covariance matrices array
    xfs[:, 0] = xfm  # All estimated target states are stored here
    Pfs[:, :, 0] = Pfiltm  # These are the corresponding covariance matrices
    r = pow(sigma_r, 2)  # measurement variance
    A = np.array([[1, 0, T, 0],  # this is the state transition matrix
                  [0, 1, 0, T],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    pred_x = 0  # reset predictions for each new range
    pred_y = 0

    if N > 1:
        for i in range(1,N):
            wt_sum = 0
            x_curr = db[i][1]  # get current auv nav_x
            y_curr = db[i][2]  # get current auv nav_y
            Xo_curr = [x_curr,y_curr]  # put auv position into format for acpkf
            Z_curr = db[i][0]  # get current range for acpkf
            T = (db[i][3]-db[i-1][3])/time_warp  # get T time since last range update
            print("Range number = "+str(i+1)+", Time since last range: " + str(round(T,1)) + " seconds")
            for t in range(num_trk):  # where num_trk is Number of filters used in (AP-CKF)
                Xthat_curr = xf_all[:, t]
                P_curr = P_all[:, :, t]
                # Now execute the cubature Kalman filter with range-only measurement
                # for the current measurement, given the current state and covariance
                # of the t-th filter
                xtf, Pfilt, likeli = ckf_ro_onestep(Xthat_curr, P_curr, Xo_curr, Z_curr, sigma_r, T, Q)
                xf_all[0, t] = xtf[0]  # not pythonic, but oh well
                xf_all[1, t] = xtf[1]  # not pythonic
                xf_all[2, t] = xtf[2]  # not pythonic
                xf_all[3, t] = xtf[3]  # not pythonic
                P_all[:, :, t] = Pfilt
                wt[t] = wt[t] * likeli
                wt_sum = wt_sum + wt[t]

            xfm = np.zeros((4))  # xfm will contain the overall state estimate     # xfm translated okay.
            Pfiltm = np.zeros((4, 4))  # Pfiltm the corresponding covariance matrix

            for jj in range(num_trk):
                wt[jj] = wt[jj] / wt_sum
                if wt[jj] < 0.001:
                    wt[jj] = 0

                xfm = xfm + wt[jj] * xf_all[:, jj]
                Pfiltm = Pfiltm + wt[jj] * P_all[:, :, jj] + wt[jj] * xf_all[:, jj] * np.transpose(xf_all[:, jj])

            xf_transpose = xfm.reshape(-1, 1)
            Pfiltm = Pfiltm - xfm * xf_transpose
            xfs[:, i] = xfm
            Pfs[:, :, i] = Pfiltm
            # plot_iter(xfs)

        Xt_hat = xfs  # prediction matrix
        # print(Xt_hat)
        pred_x = round(Xt_hat[0, N-1], 1)
        pred_y = round(Xt_hat[1, N-1], 1)

    return pred_x, pred_y


def ckf_ro_onestep(Xthat_curr, P_curr, Xo_curr, Z_curr, sigma_r, T, Q):
    '''
    Function for CKF_ro_onestep for each iteration
    No idea what it does
    '''
    r = pow(sigma_r, 2)
    A = np.array([[1, 0, T, 0],  # update state transition matrix with T
                  [0, 1, 0, T],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    nd = 4  # what's ND? size of array?
    two_nd = 2 * nd
    x_pred_pts = np.zeros((nd, two_nd))
    wts = np.ones((two_nd))
    wts = np.multiply((1 / two_nd), wts)
    xp = np.matmul(A, Xthat_curr)
    # xp = xp.reshape(-1,1)
    Pp_A_P = np.matmul(A, P_curr)
    At = np.transpose(A)
    Pp_APAt = np.matmul(Pp_A_P, At)
    Ppred = Pp_APAt + Q
    sqrt_P = np.linalg.cholesky(nd * Ppred)  # chol in matlab typically returns an upper triangular matrix,
                                            # but this returns the lower triangle as needed for apckf

    for j in range(two_nd):
        col_ind = np.mod(j, nd)
        # if col_ind == 0:
        #     col_ind = nd
        j_nd = ceil(j / nd) + 1
        sign_val = pow(-1, j_nd+1)
        # print("sign_val " + str(sign_val))
        check = sqrt_P[:, col_ind]
        x_pred_pts[:, j] = xp + np.multiply(sign_val, check)

    # now compute predicted measurement
    zp_val_vect = np.empty(two_nd)
    for j in range(two_nd):
        v = np.array([x_pred_pts[0, j] - Xo_curr[0], x_pred_pts[1, j] - Xo_curr[1]])
        zp_val = np.linalg.norm(v)
        zp_val_vect[j] = zp_val

    zp = np.matmul(zp_val_vect, np.transpose(wts))
    # compute Pxz and Pzz
    Pxz = np.zeros((nd, 1))
    Pzz = 0

    for j in range(two_nd):
        zp_val = zp_val_vect[j]
        nu = zp_val - zp
        zp_nu = nu
        first = np.multiply(wts[j], x_pred_pts[:, j] - xp)
        first = first.reshape(-1, 1)
        zp_nu_t = np.transpose(zp_nu)
        second = np.multiply(first, zp_nu_t)
        Pxz = Pxz + second
        Pzz = Pzz + wts[j] * zp_nu * np.transpose(zp_nu)

    Pzz = Pzz + r
    G = np.multiply(Pxz, np.reciprocal(Pzz))
    nu = Z_curr - zp
    nu_abs = np.abs(nu)
    xp = xp.reshape(-1,1)
    xf = xp + np.multiply(G, nu)
    Gt = np.transpose(G)
    Pfilt = Ppred - G * Pzz * np.transpose(G)
    xtf = xf
    Phat = Pfilt
    low = 1 / sqrt(2 * pi * Pzz)
    up = -0.5 * np.transpose(nu) * np.reciprocal(Pzz) * nu
    likeli = low * exp(up)

    return xtf, Phat, likeli

def log_results_csv():
    csv_temp = open(auv.csv_str, 'w', newline='')
    csvwriter = csv.writer(csv_temp)
    for i in range(len(auv.predictions_db)):
        csvwriter.writerow(auv.predictions_db[i])

    csv_temp.close()

    return

def main():
    lock = threading.Lock()
    shore = ownshipMOOS('localhost', 9000)
    auv = ownshipMOOS('localhost', 9009)
    # ship = ownshipMOOS('localhost', 9001)
    time_to_deploy = 0

    while True:
        time.sleep(1)
        if auv.show_circle:
            shore.notify("VIEW_CIRCLE", auv.range_circle, -1)  # show circle on pmarine viewr
            auv.show_circle = False

        if auv.new_range:
            N = auv.iter
            usv_starting_quarter = auv.usv_starting_quarter
            time_warp = auv.time_warp
            db = auv.track_db # db of reports
            r = auv.r # current
            x = auv.r_x
            y = auv.r_y
            print("")
            print("for N = " + str(N) + ", db: ")
            # print(db)
            lock.acquire()
            try:
                pred_x, pred_y = apckf(db, N, usv_starting_quarter, time_warp)
                print("Latest predictions, x = " + str(pred_x) + ", y =" + str(pred_y))
                if N > 1: # ignore first range only
                    auv.predictions_db.append([pred_x, pred_y])
                    marker_string = "type=diamond,x="+str(pred_x)+",y="+str(pred_y)+",scale=4,label=pred_" \
                                    +str(auv.iter)+",color=red,width=4.5"
                    shore.notify("VIEW_MARKER", marker_string, -1)
                    homing_updates_str = "points = " + str(int(pred_x)) + ", " + str(int(pred_y))
                    auv.notify("APCKF_PING", homing_updates_str, -1)
                    print(auv.predictions_db)

                    # auv.log_results_csv()

            finally:
                lock.release()
            auv.new_range = False

        elif auv.iter_acpkf:
            pass

        # if auv.publish_marker == True:
        #     marker_string = "type=diamond,x=0,y=0,scale=4,label=current_prediction,color=red,width=4.5"
        #     shore.notify("VIEW_MARKER", marker_string, -1)


if __name__ == "__main__":
    main()
