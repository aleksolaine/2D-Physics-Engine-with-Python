import math
import random
from matplotlib import pyplot as plt
from matplotlib import animation

#Global variables:
dt = 0.01 #time step between calculations
f = .99 #fake friction coefficient. 
#friction isn't actually simulated, but if f < 1, polygons will slow down on horizontal surfaces

g = -9.81 #gravitational acceleration
e = 0.7 #COR of every polygon in simulation
c = 0.001 #air resistance of every polygon in simulation

#start and end coordinates for diagonal line 1
x1a = 40.0
x1l = 0.0
y1a = 0.0
y1l = 40.0


#start and end coordinates for diagonal line 2
x2a = 60.0
x2l = 100.0
y2a = 0.0
y2l = 40.0

#y = ax + b
a1 = (x1l-x1a)/(y1l-y1a)
b1 = 40
a2 = (x2l-x2a)/(y2l-y2a)
b2 = -60

#arrays depicting the two diagonal lines
t1 = [[x1a,x1l],[y1a,y1l]]
t2 = [[x2a,x2l],[y2a,y2l]]

#normals for these lines
n1 = [x1a-x1l,y1l-y1a]
n2 = [x2a-x2l,y2l-y2a]

#unit vectors for each respective normal of a line
n1length = math.sqrt(n1[0]**2 + n1[1]**2)
n1[0] = n1[0]/n1length
n1[1] = n1[1]/n1length

n2length = math.sqrt(n2[0]**2 + n2[1]**2)
n2[0] = n2[0]/n2length
n2[1] = n2[1]/n2length

#initialise array of polygons to simulate
polygons = []

#maximun distance vector from which each polygon checks for collisions
max_d = 7.0
c_check_d = [2*max_d, 2*max_d]

#dot product function for two 2D-vectors
def dot_product(a, b):
    return a[0]*b[0]+a[1]*b[1]

class Polygon:
    
    def __init__(self, cm, index):
        self.cm = cm #coordinates of center of mass
        self.index = index #identifier of this polygon
        self.relative_points = self.generate_shape() #generate shape relative to center of mass
        self.cpm = c/self.m #calculate air resistance per mass to be used later
        self.points = self.init_points() #initialise polygon's points in world space
        target = [50.0, 40.0] #point that the polygon will start moving towards initially
        self.vx = target[0]-cm[0] #initial velocity, x-component
        self.vy = target[1]-cm[1] #initial velocity, y-component
        self.a = 0.0 #initial angular orientation
        self.w = random.uniform(-6, 6) #randomise initial angular velocity
        
    '''
    Generates the shape of the polygon
    '''
    def generate_shape(self):
        size = random.uniform(2, max_d) #How far the vertices are from center of mass
        self.m = (0.25*size)**2 #very rough estimation for mass of the polygon
        self.j = 0.5*self.m*size**2 #very rough estimation of moment of inertia
        self.collision_distance = size * -0.2 #How close to another polygon this polygon needs to be to collide
        
        vertices = random.randint(3, 10) #How many vertices the polygon has
        points = []
        #Generate vertices
        for i in range(vertices):
            points.append([size*math.cos(i*(2/vertices)*math.pi), size*math.sin(i*(2/vertices)*math.pi)])
        
        return points
        
    '''
    Initialises the polygon's points in the world reference
    '''
    def init_points(self):
        points = []
        for point in self.relative_points.copy():
            points.append([self.cm[0] + point[0], self.cm[1] + point[1]])
        return points
    
    '''
    Update is called once in every frame
    '''
    def update(self):
        #Check if the polygon collided with anything, if not, calculate new velocity in a freefall
        if not self.collision():
            self.freefall_calc_velocity()
        
        #Calculate new position for center of mass and then position of all vertices
        self.calculate_new_position()
        self.rotate_points()
        
    '''
    Checks if a collision happened and calls for appropriate function to calculate the collision
    '''
    def collision(self):
        #initialise the boolean collision to default as False
        collision = False
        
        #check if this polygon should collide with another polygon
        #iterate over all other polygons
        for poly in polygons:
            #ignore if the polygon is this one
            if poly.index != self.index:
                #ignore if distance to the other polygon is more than checking distance
                if abs(poly.cm[0]-self.cm[0]) <= c_check_d[0]:
                    if abs(poly.cm[1]-self.cm[1]) <= c_check_d[1]:
                        #potential collision, call the polygon collision function
                        collision = self.polygon_collision(poly)
        
        
        #Iterate through all points of this polygon to see if collided with environment
        for point in self.points:
            #Check if point collided with the floor, if not, did it collide with the ceiling?
            if point[1] < 0:
                # self.cm = [self.cm[0], self.cm[1] - point[1] + 0.0000001] #prevent polygon from sinking into the floor
                #r is the vector from colliding point to center of mass. Saved as instance variable to be available for use in other functions
                self.r = [point[0]-self.cm[0], point[1]-self.cm[1]]
                #point is inside floor, check if it's moving deeper into floor
                if self.point_velocity()[1] < 0:
                    #call function that updates the polygon's movement considering the collision with the floor
                    self.floor_collision_calc_velocity()
                    collision = True
            elif point[1] > 100:
                self.r = [point[0]-self.cm[0], point[1]-self.cm[1]]
                if self.point_velocity()[1] > 0:
                    self.floor_collision_calc_velocity()
                    collision = True
            
            #Check if point collided with left wall, if not, did it collide with the right wall?
            if point[0] < 0:
                self.r = [point[0]-self.cm[0], point[1]-self.cm[1]]
                if self.point_velocity()[0] < 0:
                    self.wall_collision_calc_velocity()
                    collision = True
            elif point[0] > 100:
                self.r = [point[0]-self.cm[0], point[1]-self.cm[1]]
                if self.point_velocity()[0] > 0:
                    self.wall_collision_calc_velocity()
                    collision = True
            
            #Check if point collided with either diagonal lines
            #In practice, if y < ax + b, point is behind the line
            if point[1] < (a1*point[0]+b1):
                #prevent from sinking into the incline:
                # self.cm = [self.cm[0]+n1[0]*((a1*point[0]+b1)-point[1]), self.cm[1]+n1[1]*((a1*point[0]+b1)-point[1])]
                self.r = [point[0]-self.cm[0], point[1]-self.cm[1]]
                if dot_product(self.point_velocity(), n1) < 0:
                    self.incline_collision_calc_velocity(n1)
                    collision = True
            if point[1] < (a2*point[0]+b2):
                #prevent from sinking into the incline:
                # self.cm = [self.cm[0]+n2[0]*((a2*point[0]+b2)-point[1]), self.cm[1]+n2[1]*((a2*point[0]+b2)-point[1])]
                self.r = [point[0]-self.cm[0], point[1]-self.cm[1]]
                if dot_product(self.point_velocity(), n2) < 0:
                    self.incline_collision_calc_velocity(n2)
                    collision = True
        
        #return whether collision happened or not
        return collision
    
    '''
    Called when there is another polygon close enough to potentially collide
    '''
    def polygon_collision(self, other):
        collision = False #default collision to False
        sc = len(other.points) #Side count of the other polygon
        
        #Iterate over each vertex in this polygon
        for point in self.points:
            inside = True #Assume the point is inside the colliding polygon
            
            #iterate over sides in the other polygon to find if the current point is behind that side
            for x in range(sc):
                #Side from first vertex to next one in sequence, moving counter-clockwise
                r_side = [other.points[(x+1)%(sc)][0] - other.points[x][0], other.points[(x+1)%(sc)][1] - other.points[x][1]]
                #Vector from point where collision happened to the first vertex of current side
                rp = [point[0] - other.points[x][0], point[1] - other.points[x][1]]
                #Cross product to check if the colliding point is behind this side
                #If cross product is less than a small negative number, the point is not inside
                #Value below zero was chosen instead of 0 to have a larger margin of error
                if (r_side[0]*rp[1] - rp[0]*r_side[1]) < self.collision_distance:
                    #Since the colliding point wasn't behind this side, the point isn't inside the other polygon
                    inside = False
                    break
                
            #if the point was inside the other polygon, find which side it collided with
            if inside:
                #self.r is again the vector from the polygon's center of mass to the colliding vertex
                self.r = [point[0]-self.cm[0], point[1]-self.cm[1]]
                
                d = math.inf #initialise distance to infinity
                r_colliding_side_unit = []
                #iterate over each side of other polygon
                for z in range(sc):
                    #calculate unit vector for side
                    r_side = [other.points[(z+1)%(sc)][0] - other.points[z][0], other.points[(z+1)%(sc)][1] - other.points[z][1]]
                    r_side_l = math.sqrt(r_side[0]**2 + r_side[1]**2)
                    r_side_unit = [r_side[0]/r_side_l, r_side[1]/r_side_l]
                    
                    #vector from colliding point to a point on the collided side
                    rp = [point[0] - other.points[z][0], point[1] - other.points[z][1]]
                    
                    #distance from side
                    temp_d = r_side_unit[0]*rp[1] - rp[0]*r_side_unit[1]
                    
                    #if distance is shorter than that to saved shortest side, replace and save side
                    if temp_d < d:
                        d = temp_d
                        r_colliding_side_unit = r_side_unit
                
                #calculate relative velocity of the polygons at the colliding point
                vA = self.point_velocity()
                vB = other.collision_point_velocity(point)
                vAB = [vA[0] - vB[0], vA[1] - vB[1]]
                
                #find normal to the colliding site, pointing outwards of the polygon
                n = [r_colliding_side_unit[1], -r_colliding_side_unit[0]]
                
                #find if polygons are moving towards each other
                #if they are, proceed to calculate collision
                if dot_product(vAB, n) < 0:
                    collision = True
                    
                    #precalculate levers for each polygon using cross products
                    XPa = self.r[0]*n[1]-n[0]*self.r[1]
                    XPb = other.r[0]*n[1]-n[0]*other.r[1]
                    
                    #calculate magnitude of impulse
                    I = -(1.0 + e) * dot_product(vAB, n) / (1.0/self.m + 1.0/other.m + (XPa**2)/self.j + (XPb**2)/other.j)
                        
                    #update this polygon's velocities after this collision
                    self.calc_velocity(I, n, XPa)
                    
                    #send the collision information to the other polygon
                    other.calc_velocity(-I, n, XPb)
                    
                    #collision has happened, so stop iterating over points of this polygon
                    break
        
        #return if collision happened or not
        return collision
    
    '''
    Returns absolute velocity of the point that is vector self.r away from center of mass
    '''
    def point_velocity(self):
        #rotational velocity of the point, calculated using cross product
        vr = [-self.r[1]*self.w, self.r[0]*self.w]
        vxp = self.vx + vr[0]
        vyp = self.vy + vr[1]
        
        return [vxp,vyp]
    
    '''
    This is called by another polygon, that collided with this polygon
    Returns absolute velocity of the point that is given as a paratemer
    '''
    def collision_point_velocity(self, point):
        self.r = [point[0]-self.cm[0], point[1]-self.cm[1]]
        vr = [-self.r[1]*self.w, self.r[0]*self.w]
        vxp = self.vx + vr[0]
        vyp = self.vy + vr[1]
        
        return [vxp,vyp]
    
    '''
    Update polygon's velocity in freefall
    '''
    def freefall_calc_velocity(self):
        v = math.sqrt(self.vx**2+self.vy**2)
        self.vx = self.vx - self.cpm*v*self.vx*dt
        self.vy = self.vy - self.cpm*v*self.vy*dt + g*dt
        
    '''
    Update polygon's velocity when colliding with floor
    '''
    def floor_collision_calc_velocity(self):
        
        #calculate impulse
        I = -(1.0 + e) * -self.point_velocity()[1] / (1.0/self.m + self.r[0]**2/self.j)
        
        #if impulse is very small, halve it to stop the polygon from bouncing
        if abs(I) < 0.05:
            I *= 0.5
        
        #Update vertical and rotational velocity relative to the impulse
        #Horizontal velocity will only be updated by the pseudo friction coefficient
        self.vy = self.vy - (I/self.m)
        self.vx = self.vx * f
        self.w = self.w - (I/self.j) * self.r[0]
        
    '''
    Update polygon's velocity when colliding with a wall
    '''
    def wall_collision_calc_velocity(self):
        I = -(1.0 + e) * -self.point_velocity()[0] / (1.0/self.m + self.r[1]**2/self.j)
        
        #Update only horizontal and rotational velocity
        self.vx = self.vx - (I/self.m)
        self.w = self.w - (I/self.j) * -self.r[1]
        return
    
    '''
    Update polygon's velocity when colliding with an inclined floor
    Takes the incline's normal as a parameter
    '''
    def incline_collision_calc_velocity(self, normal):
        #calculate leverage using cross product
        XP = self.r[0]*normal[1]-normal[0]*self.r[1]
        
        I = -(1.0 + e) * dot_product(self.point_velocity(), normal)  / (1.0/self.m + XP**2/self.j)
        
        #call the general velocity calculation function
        self.calc_velocity(I, normal, XP)
        return
    
    '''
    General velocity calculation after collision
    Updates all components of velocity using
    magnitude of impulse, normal vector to the collided line and the lever length
    '''
    def calc_velocity(self, I, n, XP):
        self.vy = (self.vy + (I/self.m)*n[1]) * f
        self.vx = (self.vx + (I/self.m)*n[0]) * f
        self.w = self.w + (I/self.j) * XP
    
    '''
    Updates polygon's center of mass' position in the world, knowing
    components of velocity and the time step taken
    '''
    def calculate_new_position(self):
        self.cm = [self.cm[0]+self.vx*dt, self.cm[1]+self.vy*dt]
        
    '''
    Updates position of all vertices of the polygon, knowing
    angular velocity and the time step taken
    '''
    def rotate_points(self):
        a = self.a + self.w*dt
        i = 0
        for point in self.relative_points:
            self.points[i] = [self.cm[0] + point[0] * math.cos(a) - point[1] * math.sin(a), self.cm[1] + point[0] * math.sin(a) + point[1] * math.cos(a)]
            i += 1
            
        self.a = a
        
#set up the figure, the axis, and the plot element we want to animate
fig = plt.figure()
fig.set_dpi(300)
fig.set_size_inches(5, 5)
ax = plt.axes(xlim=(0, 100), ylim=(0, 100))

#plot the inclined lines that polygons may collide with
plt.plot(t1[0],t1[1])
plt.plot(t2[0],t2[1])

patches = []
time_elapsed = 2.0
j = 0

# '''
# initialization function: plot the background of each frame
# '''
# def init():
#     return patches #empty list for now

'''
animation function calls the update function of existing polygons in every frame
'''
def animate(i):
    global time_elapsed, j
    
    #spawn a new polygon every second
    if time_elapsed >= 1.0:
        time_elapsed = 0.0
        
        #send polygons in from different sides of the area
        polygons.append(Polygon([-20 + (j*70.0)%210.0,120.0], j))
        col = [random.random(), random.random(), random.random()]
        patches.append(plt.Polygon(polygons[-1].points, color=col))
        ax.add_patch(patches[-1])
        j += 1
    
    #run the calculations for every existing polygon
    for poly in polygons:
        poly.update()
        
    #draw the updated polygons
    ind = 0
    for patch in patches:
        patch.set_xy(polygons[ind].points)
        ind += 1
    
    time_elapsed += dt
    
    return patches

#call the animator
anim = animation.FuncAnimation(fig, animate, frames=2500, interval=1000*dt, blit=True)

#save the animation
# anim.save('projekti.mp4')

plt.show()