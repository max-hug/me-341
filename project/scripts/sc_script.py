import csv

csv_file = r'C:\Users\maxhu\Documents\VS_Code\me-341\project\airfoil.txt'

with open(csv_file, 'r') as file:
    reader = csv.reader(file)
    coords = [[float(value) for value in row] for row in reader]

domain_height = 5 # [m]
domain_left = 4 # [m]
domain_right = 6 # [m]

# Delete Everything
while GetRootPart().Bodies.Count > 0:
    GetRootPart().Bodies[0].Delete()
while GetRootPart().Curves.Count > 0:
    GetRootPart().Curves[0].Delete()
while GetRootPart().DatumPlanes.Count > 0: #sketches
    GetRootPart().DatumPlanes[0].Delete()

NamedSelection.Delete("Domain", "SC-Inlet", "SC-Outlet", "SC-Airfoil", "SC-Top-Wall", "SC-Bottom-Wall")

# Start Sketching
result = ViewHelper.SetSketchPlane(Plane.PlaneXY)

# Domain
point1 = Point2D.Create(M(-domain_left),M(-2.5))
point2 = Point2D.Create(M(-domain_left),M(2.5))
point3 = Point2D.Create(M(domain_right),M(2.5))
result = SketchRectangle.Create(point1, point2, point3)

# airfoil
for i in  range (0,len(coords)-1):
    start = Point2D.Create(M(coords[i][0]), M(coords[i][1]))
    end = Point2D.Create(M(coords[i+1][0]), M(coords[i+1][1]))
    result = SketchLine.Create(start, end)
start = Point2D.Create(M(coords[-1][0]), M(coords[-1][1]))
end = Point2D.Create(M(coords[0][0]), M(coords[0][1]))
result = SketchLine.Create(start, end)

# End Sketch
mode = InteractionMode.Solid
result = ViewHelper.SetViewMode(mode)

airfoil_edges = []

for edge in GetRootPart().Bodies[0].Edges:
    if(abs(edge.EvalMid().Point[0] + domain_left) < 1e-1):
        sel = Selection.Create(edge)
        NamedSelection.Create(sel, Selection.Empty(), "SC-Inlet")
    elif(abs(edge.EvalMid().Point[0] - domain_right) < 1e-1):
        sel = Selection.Create(edge)
        NamedSelection.Create(sel, Selection.Empty(), "SC-Outlet")
    elif(abs(edge.EvalMid().Point[1] - 2.5) < 1e-1):
        sel = Selection.Create(edge)
        NamedSelection.Create(sel, Selection.Empty(), "SC-Top-Wall")
    elif(abs(edge.EvalMid().Point[1] + 2.5) < 1e-1):
        sel = Selection.Create(edge)
        NamedSelection.Create(sel, Selection.Empty(), "SC-Bottom-Wall")
    else:
        airfoil_edges.append(edge)

sel = Selection.Create(airfoil_edges)
NamedSelection.Create(sel, Selection.Empty(), "SC-Airfoil")

sel = Selection.Create(GetRootPart().Bodies[0].Faces[0])
NamedSelection.Create(sel, Selection.Empty(), "Domain")