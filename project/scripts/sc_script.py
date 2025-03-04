# Delete Objects
selection = Selection.CreateByNames("Domain")
result = Delete.Execute(selection)
selection = Selection.CreateByNames("Airfoil Curve")
result = Delete.Execute(selection)

# Insert From File
result = DocumentInsert.Execute(r"C:\Users\maxhu\Documents\VS_Code\ME_341\project\airfoil.txt")
selection = Selection.CreateByNames("Curve 1")
result = RenameObject.Execute(selection,"Airfoil Curve")
# EndBlock

# Fill
selection = Selection.CreateByNames("Domain Curve", "Airfoil Curve"))
secondarySelection = Selection.Empty()
options = FillOptions()
result = Fill.Execute(selection, secondarySelection, options, FillMode.ThreeD)
selection = Selection.CreateByNames("Surface")
result = RenameObject.Execute(selection,"Domain")
# EndBlock

# Delete Selection
selection = Face2
result = Delete.Execute(selection)
# EndBlock


# Split Faces
options = SplitFaceOptions()
selection = Face3
point = Face3.Items[0].EvalProportion(0.5, 0.5).Point
result = SplitFace.ByParametric(selection, point, FaceSplitType.UV, options, Info5)
# EndBlock

# Create Named Selection Group
primarySelection = Body1
secondarySelection = Selection.Empty()
result = NamedSelection.Create(primarySelection, secondarySelection, "Test")
# EndBlock