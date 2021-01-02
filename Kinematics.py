import matplotlib.pyplot as plt
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D

def rotateZ( theta ):
    rz = np.array( [ [math.cos( theta ), -math.sin( theta ), 0, 0 ],
                            [math.sin( theta ), math.cos( theta ), 0, 0 ],
                            [0, 0, 1, 0 ],
                            [0, 0, 0, 1 ] ] )
    return rz

def rotateY( theta ):
    ry = np.array( [ [math.cos( theta ), 0, -math.sin( theta ), 0 ],
                            [0, 1, 0, 0 ],
                            [math.sin( theta ), 0, math.cos( theta ), 0 ],
                            [0, 0, 0, 1 ] ] )
    return ry

def rotateX( theta ):
    rx = np.array( [ [1, 0, 0, 0 ],
                            [0, math.cos( theta ), -math.sin( theta ), 0 ],
                            [0, math.sin( theta ), math.cos( theta ), 0 ],
                            [0, 0, 0, 1 ] ] )
    return rx

def translate( dx, dy, dz ):
    t = np.array( [  [1, 0, 0, dx ],
                            [0, 1, 0, dy ],
                            [0, 0, 1, dz ],
                            [0, 0, 0, 1 ] ] )
    return t

def DHH( theta, d, a, alpha ):
    return rotateZ( theta ).dot( translate( 0, 0, d ) ).dot( translate( a, 0, 0 ) ).dot( rotateX( alpha ) )

def plotCoordinateSystem( ax, length = 1.0, width = 1.0, A = None ):
    if ( A is None ):
        A = np.eye( 4 )
    xAxis = np.array( [ [ 0, 0, 0, 1 ], [ length, 0, 0, 1 ] ] ).T
    yAxis = np.array( [ [ 0, 0, 0, 1 ], [ 0, length, 0, 1 ] ] ).T
    zAxis = np.array( [ [ 0, 0, 0, 1 ], [ 0, 0, length, 1 ] ] ).T

    ax.plot( A.dot( xAxis )[ 0, : ], A.dot( xAxis )[ 1, : ], A.dot( xAxis )[ 2, : ], 'r-', linewidth = width )
    ax.plot( A.dot( yAxis )[ 0, : ], A.dot( yAxis )[ 1, : ], A.dot( yAxis )[ 2, : ], 'g-', linewidth = width )
    ax.plot( A.dot( zAxis )[ 0, : ], A.dot( zAxis )[ 1, : ], A.dot( zAxis )[ 2, : ], 'b-', linewidth = width )

def drawLink( ax, A1, A2, width=2 ):
    x1 = A1.dot( np.array( [ 0, 0, 0, 1 ] ).T )
    x2 = A2.dot( np.array( [ 0, 0, 0, 1 ] ).T )
    c = np.hstack( ( x1, x2 ) ).T
    ax.plot( (c[0],c[4]), (c[1],c[5]), (c[2],c[6]), color= '#303030', linewidth = width)


def drawBody( plotco, Ori, position, flag = 1 ):
    if (flag ==1):
        plotCoordinateSystem( plotco, 1, 2, Ori )
    A_first = Ori
    A_Second = Ori
    for i in range(len(position)):
        A_Second = A_Second.dot( translate( 0, position[i][4], 0).dot(DHH( position[i][0], position[i][1], position[i][2], position[i][3] ) ))
        if (flag ==1):
            plotCoordinateSystem( plotco, 1, 2, A_Second )
            drawLink( plotco, A_first, A_Second  )
        A_first = A_Second
    end = A_first.dot( np.array([0,0,0,1]).T )
##    print(end)
    return end

def Jacobian( ax, ori, current, err ):
    J = np.zeros( (len(current), 3 ) )
    t = current.copy()
    
    for i in range( len( current ) ):
        t[i][0] += eps
        currentPp = drawBody( ax , ori, t, 0 )
        t[i][0] -= 2 * eps
        currentPn = drawBody( ax , ori, t, 0 )
        t[i][0] += eps
        J[i,:] = np.array( ( currentPp[0:3] - currentPn[0:3] ) / (2 * eps ) )

    return J

def ForwardKinematics():
    drawBody( ax, O_RightArm, K_RightArm )
    drawBody( ax, O_LeftArm, K_LeftArm )
    drawBody( ax, O_RightLeg, K_RightLeg )
    drawBody( ax, O_LeftLeg, K_LeftLeg )
    drawBody( ax, O_Head, K_Head )

def InverseKinematics(Orig, DHtable):
    homeP = drawBody( ax, Orig, DHtable, 0 )

    print('Origin', homeP )
    print('target', Inverse_solution)
    print('--------------------------------')

    current = DHtable.copy()
    
    for step in range( 200 ) :
        currentP = drawBody( ax , Orig, current, 0 )
        print(currentP)
        err = Inverse_solution - currentP
        J = Jacobian( ax, Orig, current, err )
        print('Error', err )
        deltaJ = J.dot(  err[0:3] )
        for i in range( len(current) ):
            current[i][0] += rate * deltaJ[i]

    

##  DH table = [ theta,   d,    a,   alpha,    y translate]
K_RightArm = [ [ 0.0/180.0 * math.pi, 2.0, 1.5, -90.0/180.0 * math.pi, 0.0 ],
                             [ 0.0/180.0 * math.pi, 1.5, 6.0, 90.0/180.0 * math.pi, 0.0 ],
                             [ 0.0/180.0 * math.pi, 0.0, 13, 0.0/180.0 * math.pi, -1.5] ]

K_LeftArm = [ [-90.0/180.0 * math.pi , 2.0, -1.5, -90.0/180.0 * math.pi, 0.0],
                           [ 0.0/180.0 * math.pi, 1.5, -6.0, 90.0/180.0 * math.pi , 0.0],
                           [ 0.0/180.0 * math.pi, 0.0, -13, 0.0/180.0 * math.pi, -1.5] ]

K_RightLeg = [ [ 0.0/180.0 * math.pi, 0.0, 0, 0.0/180.0 * math.pi, 0.0 ],
                            [ 0.0/180.0 * math.pi, 3.24223, 0, -90.0/180.0 * math.pi, -2.5 ],
                            [ 90.0/180.0 * math.pi, 2.5, 0, -90.0/180.0 * math.pi, 0.0 ],
                            [ 180.0/180.0 * math.pi, 0.0, 9.21875, 0.0/180.0 * math.pi, 0.0 ],
                            [ 0.0/180.0 * math.pi, 0.0, 12.65625, -90.0/180.0 * math.pi, -2.5 ],
                            [ 0.0/180.0 * math.pi, 2.5, 0, 90.0/180.0 * math.pi, 0.0 ]  ]

K_LeftLeg = [ [ 0.0/180.0 * math.pi, 0.0, 0, 0.0/180.0 * math.pi, 0.0 ],
                            [ 0.0/180.0 * math.pi, 3.24223, 0, -90.0/180.0 * math.pi, -2.5 ],
                            [ -90.0/180.0 * math.pi, 2.5, 0, -90.0/180.0 * math.pi, 0.0 ],
                            [ 180.0/180.0 * math.pi, 0.0, -9.21875, 0.0/180.0 * math.pi, 0.0 ],
                            [ 0.0/180.0 * math.pi, 0.0, -12.65625, -90.0/180.0 * math.pi, -2.5 ],
                            [ 0.0/180.0 * math.pi, 2.5, 0, 90.0/180.0 * math.pi, 0.0 ]  ]

K_Head = [ [ 90.0/180.0 * math.pi, 3.0, 0, 90.0/180.0 * math.pi, 0 ] ]

O_RightArm = translate( 5.75, 0, 10.5 ).dot( rotateY( -90.0/180.0 * math.pi ) )
O_LeftArm = translate( -5.75, 0, 10.5 ).dot( rotateY( 90.0/180.0 * math.pi ).dot( rotateZ( 90.0/180.0 * math.pi ) ) )
O_RightLeg = translate( 3.86719, 0, 0 ).dot( rotateY( 180.0/180.0 * math.pi ) )
O_LeftLeg = translate( -3.86719, 0, 0.0 ).dot( rotateY( 180.0/180.0 * math.pi ) )
O_Head = translate( 0, 0, 10.5 ).dot( rotateY( 0.0/180.0 * math.pi ) )

Inverse_solution = [5, 3, -4, 1]
rate = 0.001
eps = 0.01

fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')

X = np.array( [ 0, 1, 1, 0, 0 ] )
Y = np.array( [ 0, 0, 1, 1, 0 ] )
Z = np.array( [ 0, 0, 1, 0, 0 ] )

ax.set_xlim(-30,20)
ax.set_ylim(-30,20)
ax.set_zlim(-30,20)

ForwardKinematics()
InverseKinematics(O_LeftLeg,K_LeftLeg)

plotCoordinateSystem( ax, 2, 5 )


plt.show()
