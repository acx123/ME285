from gps3 import gps3
from collections import deque
from collections import namedtuple
import math

class GPSPath(object):
    def __init__(self,waypoints,offset=(0,0),tol=0.000015):
        for lat,lon,time in waypoints:
            self.waypoints.append((lat+offset[0],lon+offset[1],time))
    	self.pathTime = 0
        self.trackTime = 0
        self.tol = tol
        self.vectorPath = self.calcVectorFunction()
        self.currentLeg = self.vectorPath.popleft()

    #Calculate a differentiable function that returns the vector relating to the position
    #along a smooth path between waypoints at any time, t. With the startpoints at
    # (Lat,Lon) and the endpoints equal to the velocity vector at that position.
    def calcVectorFunction(self):
        Leg = namedtuple('Leg', ['vector','deltaTime','endTime'])
        pos0,time0 = self.waypoints[0]
        endTime = 0
        vec_path = list()
        for waypoint in self.waypoints:
            pos,time = waypoint
            deltaTime = time - time0
            endTime = endTime + deltaTime
            vec = Vector(pos0,pos)
            vec_path.append(Leg(vec,deltaTime,endTime))
            pos0 = pos
            time0 = time
        return deque(vec_path)

    #Return the vector from GPSInfo towards the location at path(time)
    def getVectorTo(pos,time):
        if (time <= currentLeg.endTime - currentLeg.deltaTime):
            return Vector(pos,currentLeg.vector.origin)
        elif (time >= currentLeg.endTime):
            return Vector(pos,currentLeg.vector.end)
        current_pos = time * currentLeg.vector/currentLeg.deltaTime
        return Vector(pos,current_pos.end)

    def pathDeviation(pos):
        vec = getVectorTo(pos,trackTime)
        return (abs(vec),vec.absAngle())

    #Return the time corresponding to the position that is nearest to GPSInfo along
    #the path.
    def closestPathTime(pos):
        vec = Vector(currentLeg.origin,pos.end)
        return deltaTime * currentLeg.vector * vec / (abs(vec) * abs(currentLeg.vector))

    #Return the difference between the current heading and the heading of path at trackTime
    def getRelBearing(heading):
        pathBearing = currentLeg.vector.absAngle() - heading
        if (pathBearing > 180):
            return 360 - pathBearing
        return pathBearing

    #set pathTime to path(time)
    #see if GPSInfo is near enough to the position at path(time) and see if trackTime
    #Otherwise update trackTime by some factor that represents the nearest position
    #to the path, while also taking into account the distance from the path.
    def updatePosition(pos,time):
        self.pathTime = time
        currentTime = closestPathTime(pos)
        current_vec = getVectorTo(pos, pathTime)
        trackTime = currentTime / (1 + abs(current_vec))
        if (pathTime >= endTime and abs(current_vec) <= self.tol):
            currentLeg = vectorPath.popLeft()


    #return the distance along the path between time and pathTime
	def pathDistance(time=trackTime):
		return (pathTime - time) * abs(currentLeg.vector) / currentLeg.deltaTime

	def speedReq(time=trackTime):
		return pathDistance() / (currentLeg.deltaTime - pathTime + time)

class Vector(object):
    def __init__(self,origin,end):
        self.x = end[0] - origin[0]
        self.y = end[1] - origin[1]
        self.origin = origin
        self.end = end

    def __add__(self, other):
        return Vector(self.origin + other.origin, self.end + end.y)

    def __sub__(self, other):
        return Vector(self.origin - other.origin, self.end - other.end)

    def __mul__(self, other):
        if isinstance(other, Vector):
            return self.x*other.x + self.y*other.y
        return Vector(self.origin, (self.origin[0] + self.x * other,self.origin[1] + self.y * other))

    def __abs__(self):
        return math.sqrt(self.x**2 + self.y**2)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def absAngle(self):
        return math.tan((end[1] - origin[1])/(end[0] - origin[0]))

    def __str__(self):
        return '(%g, %g) @ (%g,%g)' % (self.x, self.y,self.origin[0],self.origin[1])
