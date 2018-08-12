import sys,pygame,math,numpy
import time

pygame.init()

win = pygame.display.set_mode((700,650))

pressed = pygame.key.get_pressed()

class PhysCircle:
    
    def __init__(self, screen = pygame.display.set_mode((0,0)), color = [255,255,255],
                  center = [500,250], radius = 50, velocity = 0):
        self.screen = screen
        self.color = color
        self.center = center
        self.radius = radius
        self.velocity = velocity
    
    def draw(self):
        pygame.draw.circle(self.screen, self.color, self.center, self.radius)
        
    def remove(self):
        pygame.draw.circle(self.screen,(0,0,0), self.center, self.radius)

    def mass(self):
        mass = 3.14 * (self.radius ** 2)
        return mass
    
    @property
    def move(self):
        dt = 0.1
        dx_l = self.center[0] - self.radius
        dx_r = self.center[0] + self.radius
        dy_u = self.center[1] - self.radius
        dy_d = self.center[1] + self.radius
        
        screen_width = self.screen.get_width()
        screen_height = self.screen.get_height()
        
        if dx_l <= 0 and numpy.dot(self.velocity, [1, 0]) < 0:
            self.velocity[0] = - self.velocity[0]
        if dx_r >= screen_width and numpy.dot(self.velocity, [-1, 0]) < 0:
            self.velocity[0] = - self.velocity[0]
        if dy_u <= 0 and numpy.dot(self.velocity, [0, 1]) < 0:
            self.velocity[1] = - self.velocity[1]
        if dy_d >= screen_height and numpy.dot(self.velocity, [0, -1]) < 0:
            self.velocity[1] = - self.velocity[1]
        
        self.center[0] += int(self.velocity[0] * dt)
        self.center[1] += int(self.velocity[1] * dt)
        
        return self.center, self.velocity


def collision_circles(ob1, ob2):
    dist = numpy.array([ob2.center[0] - ob1.center[0], ob2.center[1] - ob1.center[1]])
    dist_len = numpy.dot(dist, dist) ** 0.5
    penetration_depth = dist_len - (ob1.radius + ob2.radius)
     
    if penetration_depth < 0 and (numpy.dot(ob1.velocity, dist) - numpy.dot(ob2.velocity, dist) > 0):
        v1 = numpy.array(ob1.velocity)
        v2 = numpy.array(ob2.velocity)
        v1_r = dist * numpy.dot(v1, dist) / (dist_len ** 2)
        v2_r = dist * numpy.dot(v2, dist) / (dist_len ** 2)
        v1_d = v1 - v1_r
        v2_d = v2 - v2_r

        v_r_cm = (ob1.mass() * v1_r + ob2.mass() * v2_r) / (ob1.mass() + ob2.mass())
        ob1.velocity = v1_d + 2 * v_r_cm - v1_r
        ob2.velocity = v2_d + 2 * v_r_cm - v2_r
        
    return ob1.velocity, ob2.velocity


def main_c():
    circle1 = PhysCircle(win, [255, 255, 255], [500, 250], 50, [-50, 40])
    circle2 = PhysCircle(win, [255, 255, 255], [200, 260], 50, [25, 100])
    circle3 = PhysCircle(win, [255, 255, 255], [400, 500], 100, [40, 75])
    circle4 = PhysCircle(win, [255, 255, 255], [100, 500], 75, [50, -175])
    circle5 = PhysCircle(win, [255, 255, 255], [100, 100], 50, [50, -175])

    # obj_array = [circle1,circle3,circle5]
    obj_array = [circle1, circle2, circle3, circle4, circle5]
    
    while True:
        for i in obj_array:
            i.remove()

        for i in range(len(obj_array)):
            for j in range(i + 1, len(obj_array)):
                collision_circles(obj_array[i], obj_array[j])

        for i in obj_array:
            i.move

        for i in obj_array:
            i.draw()

        pygame.display.update()
        time.sleep(1 / 23)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pass


class PolygonPhys:

    def __init__(self, screen = pygame.display.set_mode((0, 0)), color = [255, 255, 255], vertices = [],
                 velocity = [0, 0], ang_velocity = 0, mass = 0, center_mass = 0, moment_of_inertia = 0):
        self.screen = screen
        self.color = color
        self.vertices = vertices
        self.velocity = velocity
        self.ang_velocity = ang_velocity
        self.mass = mass
        self.center_mass = center_mass
        self.moment_of_inertia = moment_of_inertia
        
    def draw(self):
        vert = []
        for i in self.vertices:
            vert.append([int(round(i[0])), int(round(i[1]))])
            
        pygame.draw.lines(self.screen, self.color, True, vert)
        
    def remove(self):
        vert = []
        for i in self.vertices:
            vert.append([int(round(i[0])), int(round(i[1]))])

        pygame.draw.lines(self.screen, (0, 0, 0), True, vert)
   
    
    def get_mass(self):
        v = numpy.array(self.vertices)
        m = (numpy.cross(v[0], v[len(v) - 1]))        
        for i in range(0, len(v) - 1):
            numpy.add(m, numpy.cross(v[i + 1], v[i]), out = m)
        m = 1/2 * m
        return m
    
 
    def get_center_mass(self):
        v = numpy.array(self.vertices)
        center_mass = (numpy.cross(v[0], v[len(v) - 1])) * (v[0] + v[len(v) - 1])

        for i in range(0, len(v) - 1):
            center_mass = center_mass + (numpy.cross(v[i + 1], v[i])) * (v[i + 1] + v[i])

        center_mass[0] = 1/6 * center_mass[0] / self.mass
        center_mass[1] = 1/6 * center_mass[1] / self.mass
        return center_mass
    

    def get_moment_of_inertia(self):
        v = numpy.array(self.vertices)
        k = len(v) - 1
        inertia = (v[k][0] * v[0][1] + 2 * v[k][0] * v[k][1] + 2 * v[0][0] * v[0][1] +
                    v[0][0] * v[k][1]) * numpy.cross(v[0], v[k])

        for i in range(0, len(v) - 1):
            inertia = (v[i][0] * v[i + 1][1] + 2 * v[i][0] * v[i][1] + 2 * v[i + 1][0] * v[i + 1][1]
                       + v[i + 1][0] * v[i][1]) * numpy.cross(v[i + 1], v[i])
            
        inertia = abs(1/24 * inertia) * 3.7 # To put number at the end
        #  is not of course the right way but I am tired a bit, later I should find what is the problem
        return inertia

    
    def move(self):
        c_m = [0, 0]
        radius_vector = [0,0]        
        dt = 0.1
        c_m[0] = self.get_center_mass()[0]
        c_m[1] = self.get_center_mass()[1]
        
        for i in self.vertices:
            ang_v = - self.ang_velocity
            radius_vector[0] = i[0] - c_m[0]
            radius_vector[1] = i[1] - c_m[1]
            i[0] = c_m[0] + radius_vector[0] * math.cos(ang_v * dt) +\
                   radius_vector[1] * math.sin(ang_v * dt)
            i[1] = c_m[1] - radius_vector[0] * math.sin(ang_v * dt) +\
                   radius_vector[1] * math.cos(ang_v * dt)
            i[0] += self.velocity[0] * dt
            i[1] += self.velocity[1] * dt
            
        return self.vertices


def collision_wall(pol):
    v = numpy.array(pol.velocity)
    ang_v = pol.ang_velocity

    for i in pol.vertices:
        ec = 1
        radius_vector = numpy.array(i) - numpy.array(pol.get_center_mass())
        tang_v = numpy.array([ - ang_v * radius_vector[1], ang_v * radius_vector[0]])
        v_in_collision_point = v + tang_v

        screen_width = pol.screen.get_width()
        screen_height = pol.screen.get_height()
        walls_normals = [[1, 0], [-1, 0], [0, 1], [0, -1]]
        collisions_conditions = [i[0] <= 0, i[0] >= screen_width, i[1] <= 0, i[1] >= screen_height]

        for j in range(len(walls_normals)):
            normal = walls_normals[j]
            if collisions_conditions[j] and numpy.dot(v_in_collision_point, normal) < 0:
                impulse = ( - (1 + ec) * numpy.array(normal) * numpy.dot(v_in_collision_point, normal)) \
                          / (1 / pol.mass + 1 / pol.moment_of_inertia
                             * (numpy.cross(radius_vector, normal)) ** 2)
                pol.velocity += impulse / pol.mass
                pol.ang_velocity  += 1 / pol.moment_of_inertia * numpy.cross(radius_vector, impulse)
                #print(radius_vector, impulse)
    
    energy = pol.mass / 2 * numpy.dot(pol.velocity, pol.velocity) \
             + pol.moment_of_inertia / 2 * numpy.dot(pol.ang_velocity, pol.ang_velocity)
    #print(energy)
    
    return pol.velocity, pol.ang_velocity


def collision_response(pol1, pol2, collision_point, normal):
    ec = 1
    ang_v1 = pol1.ang_velocity
    ang_v2 = pol2.ang_velocity
    v1 = numpy.array(pol1.velocity)
    v2 = numpy.array(pol2.velocity)
    radius_vector_1 = numpy.array(collision_point) - numpy.array(pol1.get_center_mass())
    radius_vector_2 = numpy.array(collision_point) - numpy.array(pol2.get_center_mass())

    v1_in_collision_point = v1 + numpy.array([ - ang_v1 * radius_vector_1[1], 
                                               ang_v1 * radius_vector_1[0]])
    v2_in_collision_point = v2 + numpy.array([ - ang_v2 * radius_vector_2[1],
                                               ang_v2 * radius_vector_2[0]])

    print(numpy.dot(v1_in_collision_point - v2_in_collision_point, normal))
    if numpy.dot(v1_in_collision_point - v2_in_collision_point, normal) > 0:
        impulse = ( - (1 + ec) * numpy.array(normal) * numpy.dot(v1_in_collision_point 
                              - v2_in_collision_point, normal)) / (
                              1 / pol1.mass + 1 / pol2.mass + 1 / pol1.moment_of_inertia
                              * (numpy.cross(radius_vector_1, normal)) ** 2 + 1 / pol2.moment_of_inertia
                              * (numpy.cross(radius_vector_2, normal)) ** 2)

        pol1.velocity += impulse / pol1.mass
        pol1.ang_velocity += 1 / pol1.moment_of_inertia * numpy.cross(radius_vector_1, impulse)
        pol2.velocity -= impulse / pol2.mass
        pol2.ang_velocity -= 1 / pol2.moment_of_inertia * numpy.cross(radius_vector_2, impulse)
    
    energy = pol1.mass / 2 * numpy.dot(pol1.velocity, pol1.velocity) \
             + pol1.moment_of_inertia / 2 * numpy.dot(pol1.ang_velocity, pol1.ang_velocity) \
             + pol2.mass / 2 * numpy.dot(pol2.velocity, pol2.velocity) \
             + pol2.moment_of_inertia / 2 * numpy.dot(pol2.ang_velocity, pol2.ang_velocity)
    #print(energy)
    
    return pol1.velocity, pol1.ang_velocity, pol2.velocity, pol2.ang_velocity


def collision_detection(pol1, pol2):
    #Here we need to find first - collision point, second - normal, along which collision occurs
    
    #The following is calculation of the collision point
    vert1 = pol1.vertices[:]
    vert2 = pol2.vertices[:]
    axises = []
    for i in range(len(vert1)):
        if i ==  len(vert1) - 1:
            x = vert1[0][1] - vert1[i][1]
            y = vert1[i][0] - vert1[0][0]
        else:
            x = vert1[i + 1][1] - vert1[i][1]
            y = vert1[i][0] - vert1[i + 1][0]
            
        axis = numpy.array([x, y])
        axises.append(axis)
        vert1_proj = []
        
        for j in vert1:
            vert1_proj.append(numpy.dot(numpy.array(j), axis)) 
            # Dividing is not necessary 

        k = 0
        vert1_proj.sort()
        while k <= len(vert2) - 1:
            proj = numpy.dot(numpy.array(vert2[k]), axis)
            if proj < vert1_proj[0] or proj > vert1_proj[len(vert1_proj) - 1]:
                vert2.pop(k)
            else:
                k += 1
        
        if len(vert2) == 0:
            break

    if len(vert2) != 0:
        #the following is calculation of the normal vector of the collision
        vert1_proj = []        
        for j in vert1:
            vert1_proj.append(numpy.dot(numpy.array(j), axises[0]))

        vert1_proj.sort()
        normal = axises[0]
        depth = numpy.dot(numpy.array(vert2[0]), axises[0]) - vert1_proj[len(vert1_proj) - 1]
        
        for k in range(1, len(axises)):
            vert1_proj = []
            for j in vert1:
                vert1_proj.append(numpy.dot(numpy.array(j), axises[k]))
            
            vert1_proj.sort()
            d = numpy.dot(numpy.array(vert2[0]), axises[k]) - vert1_proj[len(vert1_proj) - 1]
            if d < depth:
                depth = d
                normal = axises[k]
        
        normal /= (numpy.dot(normal, normal)) ** 0.5
        #print(normal)
        collision_response(pol2, pol1, vert2[0], normal)

    return pol1.velocity, pol2.velocity, pol1.ang_velocity, pol2.ang_velocity


def main_p():
    polygon_1 = PolygonPhys(win, [250, 250, 250], [[650, 650], [650, 450],
           [500, 450]], [10, 5], 0)
    polygon_2 = PolygonPhys(win, [255, 255, 255], [[300, 300], [400, 400], [500, 300], [400, 200]],
                            [5, 50], -0.45)
    polygon_3 = PolygonPhys(win, [255, 255, 255], [[150, 150], [250, 250], [350, 150], [250, 50]],
                            [20, 50], 0)
    
    obj_array = [polygon_2, polygon_3]
    for i in obj_array:
        i.mass = i.get_mass()
        i.center_mass = i.get_center_mass()
        i.moment_of_inertia = i.get_moment_of_inertia()
        
    working = True
    while working:
        for i in obj_array:
            i.remove()

        for i in range(len(obj_array)):
            for j in range(i + 1, len(obj_array)):
                collision_detection(obj_array[i], obj_array[j])
                collision_detection(obj_array[j], obj_array[i])

        for i in obj_array:
            collision_wall(i)
            i.move()

        for i in range(len(obj_array)):
            obj_array[i].draw()
        
        pygame.display.update()
        time.sleep(1 / 23)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                working = False
                
    pygame.quit()


#main_c()
main_p()


###
