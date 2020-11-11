## PHILIPPE SAYEGH
## 260637812

import time
import random
import drawSample
import math
import _tkinter
import sys
import loader
import sys
import cPickle
import os.path
import Tkinter as tk
import sys
import Tkinter as tk

from sets import Set

from PIL import ImageFilter
import TileServer
import geoclass

debug = 0  # debug 1 or 2 means using a very simplified setup
verbose=0  # print info (can be set on command line)
versionNumber = 1.0
zoomLevel = 18
loadStateFile = 'classifier.state'  # default file for classifier data

documentation = \
"""
  This program is a stub for your COMP 417 robotics assignment.
"""




##########################################################################################
#########  Do non-stardard imports and print helpful diagnostics if necessary ############
#########  Look  for "real code" below to see where the real code goes        ############
##########################################################################################


missing = []
fix = ""

try: 
    import scipy
    from scipy import signal
except ImportError: 
    missing.append( " scipy" )
    fix = fix +  \
        """
        On Ubuntu linux you can try: sudo apt-get install python-numpy python-scipy python-matplotlib 
        On OS X you can try:  
              sudo easy_install pip
              sudo pip install  scipy
        """
try: 
    import matplotlib
    import matplotlib.pyplot as plt
except ImportError: 
    missing.append( " matplotlib" )
    fix = fix +  \
        """
        On Ubuntu linux you can try: sudo apt-get install python-matplotlib
        On OS X you can try:  
              sudo easy_install pip
              sudo pip install matplotlib
        """

try: 
    import numpy as np
except ImportError: 
     missing.append( " numpy " )
     fix = fix + \
        """
          sudo easy_install pip
          sudo pip install numpy
        """
try: 
    from sklearn.decomposition import PCA
    from sklearn.svm import SVC
except ImportError: 
     missing.append( " scikit-learn " )
     fix = fix + \
        """
          sudo easy_install pip
          sudo pip install scikit-learn
        """
try: 
    from PIL import Image
    from PIL import ImageTk
    from PIL import ImageDraw
except ImportError: 
     missing.append( " PIL (more recently known as pillow) " )
     fix = fix + \
        """
          sudo easy_install pip
          sudo pip install pillow
        """

if missing:
     print "*"*60
     print "Cannot run due to missing required libraries of modules."
     print "Missing modules: "
     for i in missing: print "    ",i
     print "*"*60
     print fix
     sys.exit(1)

version = "Greg's drone v%.1f  $HGdate: Fri, 24 Nov 2017 09:38:37 -0500 $ $Revision: f330eb3280c9 Local rev 2 $" % versionNumber

print version
print " ".join(sys.argv)
##########################################################################################
#########     Parse command-line arguments   #############################################
##########################################################################################
while len(sys.argv)>1:
    if len(sys.argv)>1 and sys.argv[1]=="-v":
        verbose = verbose+1
        del sys.argv[1]
    elif len(sys.argv)>1 and sys.argv[1]=="-load":
        if len(sys.argv)>2 and not sys.argv[2].startswith("-"):
            loadStateFile = sys.argv[2]
            del sys.argv[2]
        else:
            loadStateFile = 'classifier.state'
        del sys.argv[1]
    elif len(sys.argv)>1 and sys.argv[1] in ["-h", "-help", "--help"]: # help
        print documentation
        sys.argv[1] = "-forceusagemesasge"
    else:
        print "Unknown argument:",sys.argv[1]
        print "Usage: python ",sys.argv[0]," [-h (help)][-v]    [-f TRAININGDATADIR] [-t TESTDIR] [-load [STATEFILE]]"
        sys.exit(1)


##########################################################################################
#########  "real code" is here, at last!                                      ############
##########################################################################################
# my position
tx,ty = 0.5,0.5 # This is the translation to use to move the drone
oldp = [tx,ty]  # Last point visited

fill = "white"
image_storage = [ ] # list of image objects to avoid memory being disposed of

visited_classes = [] # list of landclasses that the drone thinks it sees during flight

visited_tiles = [] # list of the names of the tiles that the drone has visited, in order

unique_tiles = Set([]) # Set of the names of the unique tiles that have been visited

prevTilePos = [[0, 0]]

def autodraw():
    """ Automatic draw. """
    draw_objects()
    tkwindow.canvas.after(100, autodraw)


def draw_objects():
    """ Draw target balls or stuff on the screen. """
    global tx, ty, maxdx, maxdy, unmoved
    global oldp
    global objectId
    global ts # tileServer
    global actual_pX, actual_pY
    global fill
    global scalex, scaley  # scale factor between out picture and the tileServer

    global mult
    global noNew

    #tkwindow.canvas.move( objectId, int(tx-MYRADIUS)-oldp[0],int(ty-MYRADIUS)-oldp[1] )
    if unmoved: 
        # initialize on first time we get here
        unmoved=0
        tx,ty = 0,0
    else: 
        # draw the line showing the path
        tkwindow.polyline([oldp,[oldp[0]+tx,oldp[1]+ty]], style=5, tags=["path"]  )
        tkwindow.canvas.move( objectId, tx,ty )

    # update the drone position
    #print tx, ty
    oldp = [oldp[0]+tx,oldp[1]+ty]

    old_tx = tx
    old_ty = ty

    # map drone location back to lat, lon
    # This transforms pixels to WSG84 mapping, to lat,lon
    lat,lon = ts.imagePixelsToLL( actual_pX, actual_pY, zoomLevel,  oldp[0]/(256/scalex), oldp[1]/(256/scaley) )

    # get the image tile for our position, using the lat long we just recovered
    im, foox, fooy, fname = ts.tiles_as_image_from_corr(lat, lon, zoomLevel, 1, 1, 0, 0)

    unique_tiles.add(fname)

    if foox != prevTilePos[-1][0] or fooy != prevTilePos[-1][1]:
        visited_tiles.append(fname)

    prevTilePos.append([foox, fooy])


    # Use the classifier here on the image "im"
    im2 = np.asarray(im, dtype=np.float32).flatten()
    (currentTileClass, txt) = geoclass.classifyOne(pca, clf, im2)
    visited_classes.append(currentTileClass)


    # This is the drone, let's move it around
    tkwindow.canvas.itemconfig(objectId, tag='userball', fill=fill)
    tkwindow.canvas.drawn = objectId

    #  Take the tile and shrink it to go in the right place
    ## DIRECTORY CHANGED FOR WINDOWS COMPATIBILITY ##
    im = im.resize((int(im.size[0]/scalex),int(im.size[1]/scaley)))
    im.save("locationtile.gif")



    # "/tmp/locationtile.gif"
    ## DIRECTORY CHANGED FOR WINDOWS COMPATIBILITY ##
    photo = tk.PhotoImage(file="locationtile.gif" )
    tkwindow.image = tkwindow.canvas.create_image( 256/scalex*int(oldp[0]/(256/scalex)), 256/scalex*int(oldp[1]/(256/scalex)), anchor=tk.NW, image=photo, tags=["tile"] )
    image_storage.append( photo ) # need to save to avoid garbage collection




    # This arranges the stuff being shown
    tkwindow.canvas.lift( objectId )
    tkwindow.canvas.tag_lower( "tile" )
    tkwindow.canvas.tag_lower( "background" )
    tkwindow.canvas.pack()


    # Code to move the drone can go here
    # Move a small amount by changing tx,ty



    ## ---------------------------------------
    # FIRST ALGORITHM (Brownian)

    # If the current tile is urban, turn back. Else move in a random direction by at most 20 pixels.

    # if currentTileClass == 2:
    #     tx, ty = -tx, -ty
    # else:
    #    tx, ty = random.uniform(-20, 20), random.uniform(-20, 20)

    # END FIRST ALGORITHM



    ## --------------------------------------
    # SECOND ALGORITHM (smart coverage)

    # Array (pool) of possible moves: North(up), East(right), South(down) and West(left)
    moves = []

    # Moves leading to unexplored tiles are favored by increased weight

    # Checking 20 pixels in each direction to see if the tiles have already been visited


    north_lat, north_lon = ts.imagePixelsToLL(actual_pX, actual_pY, zoomLevel, oldp[0] / (256 / scalex), (oldp[1]+20) / (256 / scaley))
    east_lat, east_lon = ts.imagePixelsToLL(actual_pX, actual_pY, zoomLevel, (oldp[0]+20) / (256 / scalex), oldp[1] / (256 / scaley))
    south_lat, south_lon = ts.imagePixelsToLL(actual_pX, actual_pY, zoomLevel, oldp[0] / (256 / scalex), (oldp[1]-20) / (256 / scaley))
    west_lat, west_lon = ts.imagePixelsToLL(actual_pX, actual_pY, zoomLevel, (oldp[0]-20) / (256 / scalex), oldp[1] / (256 / scaley))

    im, foox, fooy, north_fname = ts.tiles_as_image_from_corr(north_lat, north_lon, zoomLevel, 1, 1, 0, 0)
    im, foox, fooy, east_fname = ts.tiles_as_image_from_corr(east_lat, east_lon, zoomLevel, 1, 1, 0, 0)
    im, foox, fooy, south_fname = ts.tiles_as_image_from_corr(south_lat, south_lon, zoomLevel, 1, 1, 0, 0)
    im, foox, fooy, west_fname = ts.tiles_as_image_from_corr(west_lat, west_lon, zoomLevel, 1, 1, 0, 0)

    # If any unexplored tile is detected, its direction is added to moves
    if north_fname not in unique_tiles:
        moves.append('N')

    if east_fname not in unique_tiles:
        moves.append('E')

    if south_fname not in unique_tiles:
        moves.append('S')

    if west_fname not in unique_tiles:
        moves.append('W')

    noNew = False
    if len(moves) == 0:
        noNew = True

    print moves
    if noNew == False:
        move = random.choice(moves)


    # If there isn't any unexplored tile near, pick a random direction
    if noNew:
        tx = random.uniform(-70, 70)
        ty = random.uniform(-70, 70)
    # If the chosen move is North, go in a roughly northwise direction. Next ifs are similar but for other directions
    elif move == 'N':
        tx = random.uniform(-10, 10)
        ty = random.uniform(0, 20)
    elif move == 'E':
        tx = random.uniform(0, 20)
        ty = random.uniform(-10, 10)
    elif move == 'S':
        tx = random.uniform(-10, 10)
        ty = random.uniform(-20, 0)
    elif move == 'W':
        tx = random.uniform(-20, 0)
        ty = random.uniform(-10, 10)
    else:
        print "Invalid move!:", move

    # # If the current tile is urban, abort decided move and turn back. And with an extra boost to make sure we get out of the city.
    if currentTileClass == 2:
        tx, ty = -old_tx, -old_ty

    # END SECOND ALGORITHM

    ## --------------------------------------



    # If we're heading towards an edge of the map, turn back
    if  (oldp[0] + tx) < 0 or (oldp[0] + tx) > 1024:
        tx = -tx
    if (oldp[1] + ty) < 10 or (oldp[1] + ty) > 1024:
        ty = -ty



# MAIN CODE. NO REAL NEED TO CHANGE THIS

ts = TileServer.TileServer()

# Top-left corner of region we can see

lat, lon = 45.44203, -73.602995    # verdun

# Size of region we can see, measure in 256-goepixel tiles.  Geopixel tiles are what
# Google maps, bing, etc use to represent the earth.  They make up the atlas.
#
tilesX = 20
tilesY = 20
tilesOffsetX = 0
tilesOffsetY = 0

# Get tiles to cover the whole map (do not really need these at this point, be we cache everything 
# at the biginning this way, and can draw it all.
# using 1,1 instead of tilesX, tilesY to see just the top left image as a check
#
#actual, actual_pX, actual_pY, fname = ts.tiles_as_image_from_corr(lat, lon, zoomLevel, 1, 1, tilesOffsetX, tilesOffsetY)
actual, actual_pX, actual_pY, fname = ts.tiles_as_image_from_corr(lat, lon, zoomLevel, tilesX, tilesY, tilesOffsetX, tilesOffsetY)


# Rather than draw the real data, we can use a white map to see what is unexplored.
bigpic = Image.new("RGB", (256*tilesX, 256*tilesY), "white")
bigpic.paste(actual, (0,0))  # paste the actual map over the pic.

# How to draw a rectangle.
# You should delete or comment out the next 3 lines.
#draw = ImageDraw.Draw(bigpic)
#xt,yt = 0,0
#draw.rectangle(((xt*256-1, yt*256-1), (xt*256+256+1, yt*256+256+1)), fill="red")

# put in image

# Size of our on-screen drawing is arbitrarily small
myImageSize = 1024
scalex = bigpic.size[0]/myImageSize  # scale factor between our picture and the tileServer
scaley = bigpic.size[1]/myImageSize  # scale factor between our picture and the tileServer
im = bigpic.resize((myImageSize,myImageSize))
im = im.filter(ImageFilter.BLUR)
im = im.filter(ImageFilter.BLUR)

im.save("mytemp.gif") # save the image as a GIF and re-load it does to fragile nature of Tk.PhotoImage
tkwindow  = drawSample.SelectRect(xmin=0,ymin=0,xmax=1024 ,ymax=1024, nrects=0, keepcontrol=0 )#, rescale=800/1800.)
root = tkwindow.root
root.title("Drone simulation")

# Full background image
photo = tk.PhotoImage(file="mytemp.gif")
tkwindow.imageid = tkwindow.canvas.create_image( 0, 0, anchor=tk.NW, image=photo, tags=["background"] )
image_storage.append( photo )
tkwindow.canvas.pack()

tkwindow.canvas.pack(side = "bottom", fill = "both",expand="yes")


MYRADIUS = 7
MARK="mark"

# Place our simulated drone on the map
sx,sy=600,640 # over the river
#sx,sy = 220,280 # over the canal in Verdun, mixed environment
oldp = [sx,sy]
objectId = tkwindow.canvas.create_oval(int(sx-MYRADIUS),int(sy-MYRADIUS), int(sx+MYRADIUS),int(sy+MYRADIUS),tag=MARK)
unmoved =  1

# initialize the classifier
# We can use it later using these global variables.
#
pca, clf, classnames = geoclass.loadState( loadStateFile, 1.0)

# launch the drawing thread
autodraw()

#Start the GUI
root.mainloop()

print "unique tiles visited:", len(unique_tiles)
print "visited tiles total:", len(visited_tiles)
print "visited tiles list:", visited_tiles
print "landclasses seen", len(visited_classes)