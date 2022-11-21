#ifndef OCC_H
#define OCC_H
#include <AIS_InteractiveContext.hxx>
#include <AIS_InteractiveObject.hxx>
#include <AIS_ConnectedInteractive.hxx>
#include <AIS_ListOfInteractive.hxx>
#include <AIS_ListIteratorOfListOfInteractive.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Trihedron.hxx>

#include <Aspect_Background.hxx>
#include <Aspect_TypeOfline.hxx>
#include <Aspect_WidthOfline.hxx>
#include <Aspect_Window.hxx>
#include <Bnd_Box2d.hxx>
#include <BndLib_Add2dCurve.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>

#include <BRepBuilderAPI.hxx>
#include <BRepAlgo.hxx>
#include <BRepTools.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Standard_DefineHandle.hxx>
#include <DsgPrs_LengthPresentation.hxx>
#include <GCE2d_MakeSegment.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <Geom_CartesianPoint.hxx>
#include <Geom_Axis2Placement.hxx>
#include <Geom_CartesianPoint.hxx>
#include <Geom_Line.hxx>
#include <Geom_Surface.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomTools_Curve2dSet.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <gp_Vec2d.hxx>
//#include <MMgt_TShared.hxx>
#include <Standard_Transient.hxx>
//https://dev.opencascade.org/doc/overview/html/occt__upgrade.html
#include <OSD_Environment.hxx>
#include <Precision.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_IsoAspect.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_Text.hxx>
//#include <Quantity_Factor.hxx>
//#include <Quantity_Length.hxx>
#include <Standard_Real.hxx>
#include <Quantity_NameOfColor.hxx>
//#include <Quantity_PhysicalQuantity.hxx>
//#include <Quantity_PlaneAngle.hxx>
#include <Quantity_TypeOfColor.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <SelectMgr_SelectableObject.hxx>
#include <SelectMgr_Selection.hxx>
#include <SelectMgr_SelectionManager.hxx>
#include <Standard_Boolean.hxx>
#include <Standard_CString.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Integer.hxx>
#include <Standard_IStream.hxx>
#include <Standard_Macro.hxx>
#include <Standard_NotImplemented.hxx>
#include <Standard_OStream.hxx>
#include <Standard_Real.hxx>
#include <StdPrs_Curve.hxx>
#include <StdPrs_Point.hxx>
#include <StdPrs_PoleCurve.hxx>
#include <TCollection_AsciiString.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TColgp_HArray1OfPnt2d.hxx>
#include <TCollection_AsciiString.hxx>
#include <TColStd_HSequenceOfTransient.hxx>
#include <TColStd_MapIteratorOfMapOfTransient.hxx>
#include <TColStd_MapOfTransient.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_ListIteratorOfListOfShape.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopTools_DataMapOfShapeInteger.hxx>
#include <UnitsAPI.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <WNT_Window.hxx>
#include <OpenGl_GraphicDriver.hxx>

// specific STEP
#include <STEPControl_Controller.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include "BRepGProp.hxx"
#include "BRepGProp_Face.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "StdSelect_FaceFilter.hxx"

#include <ElSLib.hxx>
#include <ProjLib.hxx>
#include "GC_MakeSegment.hxx"
#include <STEPCAFControl_Reader.hxx>
#include <XCAFApp_Application.hxx>
#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <TDataStd_TreeNode.hxx>
#include <TDataStd_Integer.hxx>
#include <TDocStd_Owner.hxx>
#include <TNaming_NamedShape.hxx>
#include <TNaming_NamedShape.hxx>
#include <TNaming_UsedShapes.hxx>
#include <XCAFDoc_Color.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_ShapeMapTool.hxx>
#include <XCAFDoc_Location.hxx>
#include <TDF_IDList.hxx>
#include <TDataStd.hxx>
#include <TDF_ChildIterator.hxx>
#include <GeomAPI_IntCS.hxx>
#include <gp_Quaternion.hxx>

#include <BRepBuilderAPI_Transform.hxx>
#include <AIS_ViewCube.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI.hxx>

#include <AIS_ConnectedInteractive.hxx>
#include <gp_Pln.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Surface.hxx>
#include <Geom_OffsetCurve.hxx>
#include <gp_Circ.hxx>

#include <Geom_Circle.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <GeomTools.hxx>
#include <BRepTools.hxx>
#include <TopLoc_Datum3D.hxx>
#include <Geom2dConvert.hxx>



#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepLib.hxx>
#include <Geom2d_OffsetCurve.hxx>
#include <GeomAPI.hxx>
#include <GeomConvert.hxx>


#include <AIS_AnimationCamera.hxx>
#include <AIS_AnimationObject.hxx>
#include <AIS_Animation.hxx>
#include <AIS_Plane.hxx>
#include<AIS_Triangulation.hxx>

#include <Poly_Connect.hxx>
#include <TColgp_Array1OfDir.hxx>
#include <CSLib_DerivativeStatus.hxx>
#include <CSLib_NormalStatus.hxx>
#include <CSLib.hxx>
#include <GeomLProp_SurfaceTool.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <TDF_Tool.hxx>


#include <IntCurvesFace_ShapeIntersector.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_BezierCurve.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom_BezierSurface.hxx>

#include <BRepAlgoAPI_Section.hxx>
#include <BRepAlgoAPI_Common.hxx>



#include <IntAna_IntQuadQuad.hxx>
#include <StlAPI_Reader.hxx>
#include <StlAPI_Writer.hxx>
#include <RWStl.hxx>
#include <RWStl_Reader.hxx>

#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TCollection_HAsciiString.hxx>
#include <Standard_Version.hxx>
#include <TDF_TagSource.hxx>
#include <XSControl_WorkSession.hxx>
#include <Transfer_ProcessForTransient.hxx>
#include <Transfer_ProcessForFinder.hxx>
#include <Transfer_TransientProcess.hxx>
#include <Message_ProgressIndicator.hxx>


#include <AIS_TextLabel.hxx>

#include <Poly_Polygon3D.hxx>
#endif // OCC_H
