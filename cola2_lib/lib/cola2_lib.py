#!/usr/bin/env python

#import pylab
import math
from numpy import *

class PID:
    # TODO: ek_1 , eik_1 i T no podrian estar dins de la propia classe???
    def __init__(self, p, i, d, fff, time_in_sec):
        if len(p) == len(i) and len(p) == len(d) and len(p) == len(fff):
            self.kp = p
            self.ti = i
            self.td = d
            self.fff = fff
            self.n = len(p)
            self.ek_1 = zeros(self.n)
            self.eik_1 = zeros(self.n)
            self.past_time = time_in_sec
        else:
            print 'ERROR: Bad vectors size!!'


    def compute(self, desired, current, time_in_sec):
        # Note: All the operations are done element by element
        T = time_in_sec - self.past_time
        self.past_time = time_in_sec
    
        # Compute errors
        if  len(desired) == self.n and len(current) == self.n:            
            ek = zeros(self.n)
            eik = zeros(self.n)
            edotk = zeros(self.n)
            
            ek = desired - current
            edotk = (ek - self.ek_1) / T
            eik = self.eik_1 + (ek * T)
            
            # Control law
            tau = zeros(self.n)
            for i in range(self.n):
                # Compute the integral part if ti > 0
                if self.ti[i] > 0.0:
                    int = (self.kp[i]/self.ti[i])*eik[i]
                    
                    # Compute tau
                    tau[i] = self.kp[i]*ek[i] + self.kp[i]*self.td[i]*edotk[i] + int + self.fff[i]
                    
                    # Anti-windup condition: if tau is saturated and ek has the 
                    # same sign than tau, eik does not increment
                    if abs(tau[i]) > 1.0 and ek[i]*tau[i] > 0:
                        eik[i] = self.eik_1[i]
                        # Because eik has been modified, recompute tau
                        int = (self.kp[i]/self.ti[i])*eik[i]
                        tau[i] = self.kp[i]*ek[i] + self.kp[i]*self.td[i]*edotk[i] + int + self.fff[i]   
                else:
                    # Compute tau without integral part
                    tau[i] = self.kp[i]*ek[i] + self.kp[i]*self.td[i]*edotk[i] + self.fff[i]
                    
            # Saturate tau
            tau = saturateValue(tau, 1.0) 
            
            self.ek_1 = ek
            self.eik_1 = eik
            return tau
        
        else:
            return None
        
        
    def reset(self, now):
        self.ek_1 = zeros(self.n)
        self.eik_1 = zeros(self.n)
        self.past_time = now
        
        
        
        
def saturateVector(v, min_max) :
    ret = zeros( len(v) )
    for i in range( len(v) ) :
        if v[i] < -min_max[i] : ret[i] = -min_max[i]
        elif v[i] > min_max[i] : ret[i] = min_max[i]
        else : ret[i] = v[i]
    return ret


def saturateValue(v, min_max) :
    ret = zeros( len(v) )
    for i in range( len(v) ) :
        if v[i] < -min_max : ret[i] = -min_max
        elif v[i] > min_max : ret[i] = min_max
        else : ret[i] = v[i]
    return ret


def saturateValueFloat(v, min_max):
    if v > min_max:
        v = min_max
    elif v < -min_max:
        v = -min_max

    return v


def computePid6Dof(desired, current, kp, ki, kd, sat, ek_1, eik_1, T):
    ek = zeros(6)
    eik = zeros(6)
    edotk = zeros(6)
    
    # All the operations are done element by element
    ek = desired - current
    edotk = (ek - ek_1) / T
    eik = eik_1 + (ek * T)
    eik = saturateVector(eik, sat)
    #print "ek: \n" + str(ek) 
    #print "eik: \n" + str(eik)
    #print "edotk: \n" + str(edotk)
    #print "ek_1: \n" + str(ek_1) 
    #print "eik_1: \n" + str(eik_1)

    # Control law
    tau = zeros(6)
    tau = kp * ek + ki * eik + kd * edotk
    tau = saturateValue(tau, 1.0) 
        
    return [tau, ek, eik]


def computePid6Dofv2(desired, current, kp, ti, td, fff, ek_1, eik_1, T):
    #Note: All the operations are done element by element
    #Compute errors
    ek = zeros(6)
    eik = zeros(6)
    edotk = zeros(6)
    
    ek = desired - current
    edotk = (ek - ek_1) / T
    eik = eik_1 + (ek * T)
    
    # Control law
    tau = zeros(6)
    for i in range(6):
        # Compute the integral part if ti > 0
        if ti[i] > 0.0:
            int = (kp[i]/ti[i])*eik[i]
            
            #Compute tau
            tau[i] = kp[i]*ek[i] + kp[i]*td[i]*edotk[i] + int + fff[i]
            
            #Anti-windup condition: if tau is saturated and ek has the same sign than tau, eik does not increment
            if abs(tau[i]) > 1.0 and ek[i]*tau[i] > 0:
                eik[i] = eik_1[i]
                
                #because eik has been modified, recompute tau
                int = (kp[i]/ti[i])*eik[i]
                tau[i] = kp[i]*ek[i] + kp[i]*td[i]*edotk[i] + int + fff[i]   
        else:
            #Compute tau without integral part
            tau[i] = kp[i]*ek[i] + kp[i]*td[i]*edotk[i] + fff[i]
            
    #Saturate tau
    tau = saturateValue(tau, 1.0) 
    
    return [tau, ek, eik, kp[0]*ek[0], (kp[0]/ti[0])*eik[0]]


def normalizeAngle(angle) :
    return (angle + ( 2.0 * math.pi * math.floor( ( math.pi - angle ) / ( 2.0 * math.pi ) ) ) )


def slopeFilter(max_slope, v, v1):
    dy = v - v1
    if dy > max_slope:
        return (v1 + max_slope)
    elif dy < -max_slope:
        return (v1 - max_slope)
    return v


def polyval(poly, f) :
    # TODO: To be matbal compatible. Make it more efficient!
    p = list(poly)
    p.reverse()
    
    value = 0.0
    change_sign = False
    ret = 0
    
    if f < 0.0:
        value = abs(f)
        change_sign = True
    else :
        value = f
        change_sign = False
    
    for i in range(len(p)) :
        ret = ret + pow(value, i) * p[i]
        
    if change_sign: 
        ret = ret * -1.0
        
    return ret
    

def test() :
    print "SaturateVector:"
    print str(saturateVector([1.8,0.3,-3.2, -0.7], ones(4)))
    
    print "SaturateValue:"
    print str(saturateValue([1.8,0.3,-3.2, -0.7], 1.0))    

    print "Normalize angle 7.14 = " + str(normalizeAngle(7.14))

if __name__ == '__main__':
    test()


