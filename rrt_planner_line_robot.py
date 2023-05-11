import time
import random
import drawSample
import math
import sys
import imageToRects
import utils

args = utils.get_args()
visualize = utils.get_args()
drawInterval = 100  # 10 is good for normal real-time drawing

prompt_before_next = 1  # ask before re-running sonce solved
SMALLSTEP = args.step_size  # what our "local planner" can handle.
map_size, obstacles = imageToRects.imageToRects(args.world)
# Note the obstacles are the two corner points of a rectangle
# Each obstacle is (x1,y1), (x2,y2), making for 4 points
XMAX = map_size[0]
YMAX = map_size[1]

G = [[0], []]  # nodes, edges
vertices = [
    [args.start_pos_x, args.start_pos_y],
    [args.start_pos_x, args.start_pos_y + 10],
]

# Robot length
robot_len = args.robot_length

# goal/target
tx = args.target_pos_x
ty = args.target_pos_y

# start
sigmax_for_randgen = XMAX / 2.0
sigmay_for_randgen = YMAX / 2.0
nodes = 0
edges = 1
curr_theta = args.start_pos_theta


def redraw(canvas):
    canvas.clear()
    canvas.markit(tx, ty, r=SMALLSTEP)
    drawGraph(G, canvas)
    for o in obstacles:
        canvas.showRect(o, outline="blue", fill="blue")
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices, nodes, edges
    if not visualize:
        return
    for i in G[edges]:
        canvas.polyline([vertices[i[0]], vertices[i[1]]])


# Draw line robot on canvas, at point p with orientation theta
def drawLineRobot(canvas, p, theta):
    global robot_len
    canvas.polyline(
        [p, [p[0] + robot_len * math.cos(theta), p[1] + robot_len * math.sin(theta)]],
        style=1,
    )


def genPoint():
    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random() * XMAX
            y = random.random() * YMAX
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
        else:
            print("Not yet implemented")
            quit(1)
        # range check for gaussian
        if x < 0:
            bad = 1
        if y < 0:
            bad = 1
        if x > XMAX:
            bad = 1
        if y > YMAX:
            bad = 1
    return [x, y]


def returnParent(k, canvas):
    """Return parent note for input node k."""
    for e in G[edges]:
        if e[1] == k:
            canvas.polyline([vertices[e[0]], vertices[e[1]]], style=3)
            return e[0]


def genvertex():
    vertices.append(genPoint())
    return len(vertices) - 1


def pointToVertex(p):
    vertices.append(p)
    return len(vertices) - 1


def pickvertex():
    return random.choice(range(len(vertices)))


def pointPointDistance(p1, p2):
    d = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
    return d


def closestPointToPoint(G, p2):
    # return vertex index
    closestN = -1
    closestD = XMAX * YMAX
    for i in range(len(G[nodes])):
        currD = pointPointDistance(vertices[i], p2)
        if currD < closestD:
            closestD = currD
            closestN = i
    return closestN


# This function determines the orientation of three ordered points
# returns (int):
#   - 0 if clockwise
#   - 1 if counter-clockwise
#   - 2 if collinear
def triplePointOrientation(p1, p2, p3):
    # compare slopes
    # p, q, r
    # (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
    d = float(p2[1] - p1[1]) * (p3[0] - p2[0]) - float(p2[0] - p1[0]) * (p3[1] - p2[1])

    if d > 0:  # clockwise
        return 0
    elif d < 0:  # counter-clockwise
        return 1
    else:  # collinear
        return 2


# Given three collinear points, check if p2 lies on line segment p1--p3
def onLine(p1, p2, p3):
    if (
        (p2[0] <= max(p1[0], p3[0]))
        and (p2[0] >= min(p1[0], p3[0]))
        and (p2[1] <= max(p1[1], p3[1]))
        and (p2[1] >= min(p1[1], p3[1]))
    ):
        return True
    return False


# Check if two lines intersect
# line1: p11 -- p12
# line2: p21 -- p22
def linesIntersect(p11, p12, p21, p22):
    # 2 conditions to verify (lines intersect iff one of the 2 conditions are met)
    # Condition 1:
    #   - (p11, p12, p21) and (p11, p12, p22) have different orientations AND
    #   - (p21, p22, p11) and (p21, p22, p12) have different orientations
    # Condition 2:
    #   - (p11, p12, p21), (p11, p12, p21), (p21, p22, p11), and (p21, p22, p12) are all collinear AND
    #   - the x-projections of both lines intersect
    #   - the y-projections of both lines intersect

    # Find the 4 important orientations
    orient1 = triplePointOrientation(p11, p12, p21)
    orient2 = triplePointOrientation(p11, p12, p22)
    orient3 = triplePointOrientation(p21, p22, p11)
    orient4 = triplePointOrientation(p21, p22, p12)

    # Check Condition 1
    if (orient1 != orient2) and (orient3 != orient4):
        return True

    # Check Condition 2
    # p11, p12, p21 are collinear and p21 lies on line 1
    if (orient1 == 0) and onLine(p11, p21, p12):
        return True

    # p11, p12, p22 are collinear and p22 lies on line 1
    if (orient2 == 0) and onLine(p11, p22, p12):
        return True

    # p21, p22, p11 are collinear and p11 lies on line 2
    if (orient3 == 0) and onLine(p21, p11, p22):
        return True

    # p21, p22, p12 are collinear and p12 lies on line 2
    if (orient4 == 0) and onLine(p21, p12, p22):
        return True

    return False


# Credit to https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
# which provides an algorithm for checking if two line segments interect
def lineHitsRect(p1, p2, r):
    # Check if line intersects with any of the rectangle's edges
    rtl = [r[0], r[1]]  # r's top left point
    rtr = [r[2], r[1]]  # r's top right point
    rbl = [r[0], r[3]]  # r's bottom left point
    rbr = [r[2], r[3]]  # r's bottom right point

    # Check intersection with r's left edge
    if linesIntersect(p1, p2, rbl, rtl):
        return True
    # Check intersection with r's top edge
    elif linesIntersect(p1, p2, rtl, rtr):
        return True
    # Check intersection with r's right edge
    elif linesIntersect(p1, p2, rtr, rbr):
        return True
    # Check intersection with r's bottom edge
    elif linesIntersect(p1, p2, rbr, rbl):
        return True

    return False


# Check if point p is within the rectagle, dialed with dilation
def inRect(p, rect, dilation):
    if p[0] < rect[0] - dilation:
        return 0
    if p[1] < rect[1] - dilation:
        return 0
    if p[0] > rect[2] + dilation:
        return 0
    if p[1] > rect[3] + dilation:
        return 0
    return 1


def addNewPoint(p1, p2, stepsize):
    # return vertex index
    angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])  # angle = arctan((y2-y1)/(x2-x1))
    xdelta = stepsize * math.cos(angle)
    ydelta = stepsize * math.sin(angle)
    xnew = p1[0] + xdelta  # xnew = x1 + xdelta
    ynew = p1[1] + ydelta  # ynew = y1 + xdelta
    newP = [xnew, ynew]
    return pointToVertex(newP)


def rrt_search(G, tx, ty, canvas):
    # Fill this function as needed to work ...

    global sigmax_for_randgen, sigmay_for_randgen
    global robot_len
    n = 0
    nsteps = 0
    while 1:
        p = genPoint()

        cp = closestPointToPoint(G, p)  # index format
        v = addNewPoint(vertices[cp], p, SMALLSTEP)

        if visualize:
            n = n + 1
            if n > 10:
                canvas.events()
                n = 0

        reject = False
        for o in obstacles:
            if lineHitsRect(vertices[cp], vertices[v], o) or inRect(
                vertices[v], o, robot_len
            ):
                reject = True
                vertices.pop()
                break

        if reject:
            continue

        # turn robot towards guiding point
        curr_theta = math.atan2(p[1] - vertices[cp][1], p[0] - vertices[cp][0])

        G[nodes].append(v)
        G[edges].append((cp, v))
        nsteps = nsteps + 1
        if visualize:
            drawLineRobot(canvas, vertices[cp], curr_theta)
            canvas.polyline([vertices[cp], vertices[v]])

        # Check if either end of the robot has reached the target
        if (
            pointPointDistance(vertices[v], [tx, ty]) < SMALLSTEP
            or pointPointDistance(
                [
                    vertices[v][0] + robot_len * math.cos(curr_theta),
                    vertices[v][1] + robot_len * math.sin(curr_theta),
                ],
                [tx, ty],
            )
            < SMALLSTEP
        ):
            print("Target achieved.", nsteps, "nodes in entire tree")
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((v, t))
                if visualize:
                    canvas.polyline([vertices[v], vertices[t]], 1)
                nsteps = 0
                totaldist = 0
                k = v
                while 1:
                    oldp = vertices[k]  # remember point to compute distance
                    k = returnParent(k, canvas)  # follow links back to root.
                    canvas.events()
                    if k <= 1:
                        break  # have we arrived?
                    nsteps = nsteps + 1  # count steps
                    totaldist = totaldist + pointPointDistance(
                        vertices[k], oldp
                    )  # sum lengths
                print("Path length", totaldist, "using", nsteps, "nodes.")

                global prompt_before_next
                if prompt_before_next:
                    canvas.events()
                    print("More [c,q,g,Y]>")
                    d = sys.stdin.readline().strip().lstrip()
                    print("[" + d + "]")
                    if d == "c":
                        canvas.delete()
                    if d == "q":
                        return
                    if d == "g":
                        prompt_before_next = 0
                break


def main():
    # seed
    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(
            xmin=0, ymin=0, xmax=XMAX, ymax=YMAX, nrects=0, keepcontrol=0
        )
        for o in obstacles:
            canvas.showRect(o, outline="red", fill="blue")
    while 1:
        redraw(canvas)
        G[edges].append((0, 1))
        G[nodes].append(1)
        if visualize:
            canvas.markit(tx, ty, r=SMALLSTEP)

        drawGraph(G, canvas)
        drawLineRobot(canvas, vertices[0], curr_theta)
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()


if __name__ == "__main__":
    main()
