from tkinter import Tk, Canvas, PhotoImage
import time
import numpy as np

tk = Tk()
canvas = Canvas(tk, width=1000, height=1000)
canvasicon = Canvas(tk, width=300, height=300)
canvaslogo = Canvas(tk, width=300, height=300)
canvasstat = Canvas(tk, width=300, height=300)
canvas.grid(column=0, row=0)
canvasicon.grid(column=1, row=0)
canvaslogo.place(x=1000, y=0)
canvasstat.place(x=1000, y=700)
tk.update_idletasks()
tk.update()
contact_bool = False
launch_bool = False
body_landed = None
dist_body_landed = None
closest_body = None
crash = False
center_shiftx = 0
center_shifty = 0
total_shiftx = 0
total_shifty = 0
contact_number = 0
gravity_constant = 6.67384 * (10**-11)
orbital_velocity_text = canvasstat.create_text(155, 100, text=None, font=('Courier', 10))
velocity_text = canvasstat.create_text(80, 150, text=None, font=('Courier', 10))
name_text = canvasstat.create_text(97, 50, text=None, font=('Courier', 10))
logo = PhotoImage(file='logo.gif')
bkgd = PhotoImage(file='bkgd.gif')
canvaslogo.create_image(0, 0, image=logo, anchor='nw')
canvas.create_image(0, 0, image=bkgd, anchor='nw')


class Body:
    def __init__(self, canvas, mass, coords, color):
        self.canvas = canvas
        self.coords = np.array(coords)
        self.radius = np.power(mass, 1/3) * .0008
        if self.radius < 10:
            self.radius = 10
        self.velocity = np.zeros(2)
        self.mass = mass
        self.force = np.zeros(2)
        self.acceleration = np.zeros(2)
        self.id = canvas.create_oval(self.coords[0] - self.radius, self.coords[1] - self.radius,
                                     self.coords[0] + self.radius, self.coords[1] + self.radius, fill=color)

    def go_up(self, evt):
        if contact_bool == False:
            self.force += [0, -1000]

    def go_down(self, evt):
        if contact_bool == False:
            self.force += [0, 1000]

    def go_left(self, evt):
        if contact_bool == False:
            self.force += [-1000, 0]

    def go_right(self, evt):
        if contact_bool == False:
            self.force += [1000, 0]

    def launch(self, evt):
        global body_landed
        global launch_bool
        launch_bool = True
        body_landed = None

    def draw(self):
        canvas.coords(self.id, self.coords[0] - self.radius, self.coords[1] - self.radius,
                      self.coords[0] + self.radius, self.coords[1] + self.radius)

    def update(self, dt, init_vel):
        global crash
        global contact_bool
        init_vel = np.array(init_vel)
        self.acceleration = self.force/self.mass
        old_coords = self.coords
        self.coords = 0.5 * self.acceleration * dt * dt + init_vel * dt + self.coords
        if contact_bool and fast(bodies, mainbody):
            #print(contact(bodies, mainbody))
            crash = True
        self.velocity = (self.coords - old_coords)/dt


class Icon:
    def __init__(self, canvas, body, color):
        self.icon_radius = body.radius*15/1000 if body.radius > 66 else 2
        self.icon = canvasicon.create_oval(body.coords[0]*15/1000 + 150 - self.icon_radius,
                                           body.coords[1]*15/1000 + 150 - self.icon_radius,
                                           body.coords[0]*15/1000 + 150 + self.icon_radius,
                                           body.coords[1]*15/1000 + 150 + self.icon_radius, fill=color)

    def draw(self, body):
        global total_shiftx
        global total_shifty
        canvasicon.coords(self.icon,
                          body.coords[0]*15/1000 + 150 - self.icon_radius - total_shiftx*0.015,
                          body.coords[1]*15/1000 + 150 - self.icon_radius - total_shifty*0.015,
                          body.coords[0]*15/1000 + 150 + self.icon_radius - total_shiftx*0.015,
                          body.coords[1]*15/1000 + 150 + self.icon_radius - total_shifty*0.015)


def update_forces(bodies, mainbody):
    global body_landed
    global crash
    for bd1 in bodies:
        for bd2 in bodies:
            if bd1 == bd2:
                break
            dist = bd2.coords - bd1.coords
            force = gravity_constant*bd1.mass*bd2.mass*dist/(np.linalg.norm(dist)**3)
            bd1.force += force
            bd2.force -= force


def center(bodies):
    global total_shiftx
    global total_shifty
    global centered_body
    global center_shiftx
    global center_shifty
    for bd in bodies:
        center_shiftx = 500 - centered_body.coords[0]
        bd.coords[0] += 500 - centered_body.coords[0]
        center_shifty = 500 - centered_body.coords[1]
        bd.coords[1] += 500 - centered_body.coords[1]
    total_shiftx += center_shiftx
    total_shifty += center_shifty


def make_velocity_zero(mainbody):
        global body_landed
        global dist_body_landed
        mainbody.coords = body_landed.coords + dist_body_landed
        mainbody.velocity = body_landed.velocity


def contact(bodies, mainbody):
    global contact_bool
    global body_landed
    global dist_body_landed
    global contact_number
    for bd in bodies:
        if np.linalg.norm(mainbody.coords - bd.coords) <= bd.radius + mainbody.radius and bd is not mainbody:
            contact_bool = True
            body_landed = bd
            contact_number += 1
            if contact_number == 1:
                dist_body_landed = mainbody.coords - body_landed.coords
            return True
    contact_bool = False
    return False


def fast(bodies, mainbody):
    global closest_body
    dist_dict = {}
    closest_body = None
    for bd in bodies:
        dist_to_obj = np.linalg.norm(mainbody.coords - bd.coords)
        if bd is not mainbody:
            # dist_dict[bd] = np.linalg.norm(dist_to_obj) - bd.radius
            if dist_to_obj < 3*bd.radius:
                closest_body = bd
    if closest_body is None:
        closest_body = star
    if np.linalg.norm(mainbody.velocity - closest_body.velocity) >= 50:
        return True
    return False


def lift_off(mainbody):
    global body_landed
    global dist_body_landed
    init_vel_dict[mainbody] = body_landed.velocity
    mainbody.force = dist_body_landed/np.linalg.norm(dist_body_landed) * 10000


def show_velocity(mainbody):
    global closest_body
    global contact_bool
    dist = np.linalg.norm(mainbody.coords - closest_body.coords)
    orb_vel = int(0.7*(gravity_constant*(mainbody.mass + closest_body.mass)/dist)**0.5)
    canvasstat.itemconfig(orbital_velocity_text, text='Speed for circular orbit: '+str(orb_vel))
    vel = int(np.linalg.norm(mainbody.velocity - closest_body.velocity))
    if contact_bool == True:
        vel = 0
    canvasstat.itemconfig(velocity_text, text='Your speed: '+str(vel))

def name_body():
    global closest_body
    global contact_bool
    bod_dict = {star: 'Rega', planet: 'Accordia', planet2: 'Discordia'}
    if contact_bool == False:
        canvasstat.itemconfig(name_text, text='Orbiting ' + bod_dict[closest_body])
    else:
        canvasstat.itemconfig(name_text, text='Landed on ' + bod_dict[closest_body])


rocket = Body(canvas, 5, [478.0, 9000.0], '#fafabb')
planet = Body(canvas, 2*10**17, [0.0, 9000.0], 'blue')
planet2 = Body(canvas, 2*10**17, [0.0, -9000.0], 'red')
star = Body(canvas, 1*10**19, [0.0, 0.0], 'yellow')
rocket_icon = Icon(canvas, rocket, 'black')
star_icon = Icon(canvas, star, 'yellow')
planet_icon = Icon(canvas, planet, 'blue')
planet2_icon = Icon(canvas, planet2, 'red')
icons_dict = {rocket: rocket_icon, star: star_icon, planet2: planet2_icon, planet: planet_icon}
bodies = [planet, star, planet2, rocket]
init_vel_dict = {planet: [190.0, 0.0], star: [0.0, 0.0], planet2: [-190.0, 0.0], rocket: [190, 0.0]}
dt = 0.01
mainbody = rocket
centered_body = rocket
canvas.bind_all('<KeyPress-Up>', mainbody.go_down)
canvas.bind_all('<KeyPress-Down>', mainbody.go_up)
canvas.bind_all('<KeyPress-Left>', mainbody.go_left)
canvas.bind_all('<KeyPress-Right>', mainbody.go_right)
canvas.bind_all('<space>', mainbody.launch)

for x in range(20):
    for y in range(20):
        canvasicon.create_rectangle(x*15, y*15, (x + 1)*15, (y + 1)*15)


while True:
    contact(bodies, mainbody)
    fast(bodies, mainbody)
    if contact_bool == False and crash == False:
        for eachbod in bodies:
            eachbod.draw()
            eachbod.update(dt, init_vel_dict[eachbod])
            icons_dict[eachbod].draw(eachbod)
        show_velocity(mainbody)
        name_body()
        update_forces(bodies, mainbody)
        center(bodies)
        tk.update_idletasks()
        tk.update()
        time.sleep(dt)

    elif contact_bool == True and crash == False:
        for eachbod in bodies:
            eachbod.draw()
            eachbod.update(dt, init_vel_dict[eachbod])
            icons_dict[eachbod].draw(eachbod)

        make_velocity_zero(mainbody)
        show_velocity(mainbody)
        name_body()
        update_forces(bodies, mainbody)
        center(bodies)
        tk.update_idletasks()
        tk.update()
        time.sleep(dt)

        if launch_bool == True:
            for i in range(100):
                contact(bodies, mainbody)
                for eachbod in bodies:
                    eachbod.draw()
                    eachbod.update(dt, init_vel_dict[eachbod])

                show_velocity(mainbody)
                name_body()
                update_forces(bodies, mainbody)
                center(bodies)
                lift_off(mainbody)
                tk.update_idletasks()
                tk.update()
                time.sleep(dt)
            contact(bodies, mainbody)
            launch_bool = False
            contact_bool = False
            crash = False
            contact_number = 0

    elif crash == True:
        print('You Crashed!')
        break
