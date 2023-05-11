using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Runtime;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace NotcherSuite
{
    //Provides several commands to apply notches to a pattern piece
    //1. Creates a notch CW or CCW from a point on a poly line a specified distance along the perimeter
    //2. Creates notches from one point to the next point at specified distance intervals
    //3. Creates notches from one point to the next point along specified divisions
    //4. Creates a notch on one polyline from a base point (CW or CCW), using the distance 
    //along the perimeter betwen two points of another polyline

    //might also rotate the notch after insertion
    //ERRORS
    //Need to adjust to handle if the user doesnt click on the polyline for endpoints
    public class Class1
    {
        [CommandMethod("Notch1")]
        public void notch()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;
            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                BlockTable bt = tr.GetObject(db.BlockTableId, OpenMode.ForRead) as BlockTable;
                //request polyline
                //filter for only polylines
                PromptEntityOptions peo = new PromptEntityOptions("\nPlease choose a polyine.");
                peo.SetRejectMessage("\nPlease be sure to click a polyline.");
                peo.AddAllowedClass(typeof(Polyline), true);
                Polyline pLine;
                //get the polyline
                PromptEntityResult per = ed.GetEntity(peo);
                if (per.Status == PromptStatus.OK)
                { pLine = tr.GetObject(per.ObjectId, OpenMode.ForRead) as Polyline; }
                else
                    return;//if they cancel or select something else, then back out

                //request basepoint
                PromptPointOptions ppo = new PromptPointOptions("\nSelect basepoint.");
                //request points for segment
                Point3d first = new Point3d();
                PromptPointResult ppr = ed.GetPoint(ppo);
                if (ppr.Status == PromptStatus.OK)
                { first = ppr.Value; }
                else
                { return; }

                //request distance
                double distance;
                PromptDoubleOptions pdo = new PromptDoubleOptions("\nDistance: ");
                PromptDoubleResult pdr = ed.GetDouble(pdo);
                if (pdr.Status == PromptStatus.OK)
                { distance = Math.Abs(pdr.Value); }
                else
                    return;

                //specify CW or CCW
                bool CW = true;
                PromptKeywordOptions pko = new PromptKeywordOptions("\nClockWise or Counter ClockWise from basepoint?");
                pko.Keywords.Add("CW");
                pko.Keywords.Add("CCW");
                PromptResult pRes = ed.GetKeywords(pko);
                if (pRes.Status == PromptStatus.OK)
                {
                    if (pRes.StringResult.ToUpper() == "CW")
                    { CW = true; }
                    else if (pRes.StringResult.ToUpper() == "CCW")
                    { CW = false; }
                    else
                        return;
                }
                else
                    return;

                //find point along perimter
                Point3d insertPoint = findWalkPoint(first, pLine, CW, distance);

                //insert notch at point             
                blockInserter(bt, db, insertPoint);

                tr.Commit();
            }
        }

        [CommandMethod("Notch2")]
        public void notchDist()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;
            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                BlockTable bt = tr.GetObject(db.BlockTableId, OpenMode.ForRead) as BlockTable;

                //request polyline
                //filter for only polylines
                PromptEntityOptions peo = new PromptEntityOptions("\nPlease choose a polyine.");
                peo.SetRejectMessage("\nPlease be sure to click a polyline.");
                peo.AddAllowedClass(typeof(Polyline), true);
                Polyline pLine;
                //get the polyline
                PromptEntityResult per = ed.GetEntity(peo);
                if (per.Status == PromptStatus.OK)
                { pLine = tr.GetObject(per.ObjectId, OpenMode.ForRead) as Polyline; }
                else
                    return;//if they cancel or select something else, then back out

                //request basepoint
                PromptPointOptions ppo = new PromptPointOptions("\nSelect basepoint.");
                //request points for segment
                Point3d first = new Point3d();
                PromptPointResult ppr = ed.GetPoint(ppo);
                if (ppr.Status == PromptStatus.OK)
                { first = ppr.Value; }
                else
                { return; }

                //request endpoint
                Point3d second = new Point3d();
                ppo.Message = "\nSelect endpoint.";
                ppr = ed.GetPoint(ppo);
                if(ppr.Status == PromptStatus.OK)
                { second = ppr.Value; }
                else
                { return; }

                //request distance for notches
                double distance;
                PromptDoubleOptions pdo = new PromptDoubleOptions("\nNotch every (x) inches: ");
                PromptDoubleResult pdr = ed.GetDouble(pdo);
                if (pdr.Status == PromptStatus.OK)
                { distance = Math.Abs(pdr.Value); }
                else
                    return;

                //specify CW or CCW
                bool CW = true;
                PromptKeywordOptions pko = new PromptKeywordOptions("\nClockWise or Counter ClockWise from basepoint?");
                pko.Keywords.Add("CW");
                pko.Keywords.Add("CCW");
                PromptResult pRes = ed.GetKeywords(pko);
                if (pRes.Status == PromptStatus.OK)
                {
                    if (pRes.StringResult.ToUpper() == "CW")
                    { CW = true; }
                    else if (pRes.StringResult.ToUpper() == "CCW")
                    { CW = false; }
                    else
                        return;
                }
                else
                    return;

                //get distance between points
                double totalDistance = findDistance(first, second, pLine);

                //loop through polyline segments for distance multiples
                List<Point3d> insertions = new List<Point3d>();
                Point3d insertPoint = new Point3d();
                for (double i = distance; i <= totalDistance; i += distance)
                {
                    insertPoint = findWalkPoint(first, pLine, CW, i);
                    insertions.Add(insertPoint);
                }

                //insert notches at points
                foreach (Point3d p in insertions)
                { blockInserter(bt, db, p); }

                tr.Commit();
            }
        }

        [CommandMethod("Notch3")]
        public void notchdivision()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;
            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                BlockTable bt = tr.GetObject(db.BlockTableId, OpenMode.ForRead) as BlockTable;

                //request polyline
                //filter for only polylines
                PromptEntityOptions peo = new PromptEntityOptions("\nPlease choose a polyine.");
                peo.SetRejectMessage("\nPlease be sure to click a polyline.");
                peo.AddAllowedClass(typeof(Polyline), true);
                Polyline pLine;
                //get the polyline
                PromptEntityResult per = ed.GetEntity(peo);
                if (per.Status == PromptStatus.OK)
                { pLine = tr.GetObject(per.ObjectId, OpenMode.ForRead) as Polyline; }
                else
                    return;//if they cancel or select something else, then back out

                //request basepoint
                PromptPointOptions ppo = new PromptPointOptions("\nSelect basepoint.");
                //request points for segment
                Point3d first = new Point3d();
                PromptPointResult ppr = ed.GetPoint(ppo);
                if (ppr.Status == PromptStatus.OK)
                { first = ppr.Value; }
                else
                { return; }

                //request endpoint
                Point3d second = new Point3d();
                ppo.Message = "\nSelect endpoint.";
                ppr = ed.GetPoint(ppo);
                if (ppr.Status == PromptStatus.OK)
                { second = ppr.Value; }
                else
                { return; }

                //request divisor
                double divisor;
                PromptDoubleOptions pdo = new PromptDoubleOptions("\nDivisions: ");
                PromptDoubleResult pdr = ed.GetDouble(pdo);
                if (pdr.Status == PromptStatus.OK)
                { divisor = Math.Abs(pdr.Value); }
                else
                    return;

                //specify CW or CCW
                bool CW = true;
                PromptKeywordOptions pko = new PromptKeywordOptions("\nClockWise or Counter ClockWise from basepoint?");
                pko.Keywords.Add("CW");
                pko.Keywords.Add("CCW");
                PromptResult pRes = ed.GetKeywords(pko);
                if (pRes.Status == PromptStatus.OK)
                {
                    if (pRes.StringResult.ToUpper() == "CW")
                    { CW = true; }
                    else if (pRes.StringResult.ToUpper() == "CCW")
                    { CW = false; }
                    else
                        return;
                }
                else
                    return;

                //calculate distance between points
                double totalDistance = findDistance(first, second, pLine);

                //divide total for an incremental segment
                double distance = totalDistance / divisor;

                //loop through polyline segments for distance multiples
                List<Point3d> insertions = new List<Point3d>();
                Point3d insertPoint = new Point3d();
                for (double i = distance; i <= totalDistance; i += distance)
                {
                    insertPoint = findWalkPoint(first, pLine, CW, i);
                    insertions.Add(insertPoint);
                }

                //notch the first and last points
                blockInserter(bt, db, first);
                blockInserter(bt, db, second);

                //insert notches at points
                foreach (Point3d p in insertions)
                { blockInserter(bt, db, p); }

                tr.Commit();
            }
        }

        [CommandMethod("Notch4")]
        public void notchref()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;
            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                BlockTable bt = tr.GetObject(db.BlockTableId, OpenMode.ForRead) as BlockTable;

                //request polyline
                //filter for only polylines
                PromptEntityOptions peo = new PromptEntityOptions("\nPlease choose a polyine.");
                peo.SetRejectMessage("\nPlease be sure to click a polyline.");
                peo.AddAllowedClass(typeof(Polyline), true);
                Polyline pLine;
                //get the polyline
                PromptEntityResult per = ed.GetEntity(peo);
                if (per.Status == PromptStatus.OK)
                { pLine = tr.GetObject(per.ObjectId, OpenMode.ForRead) as Polyline; }
                else
                    return;//if they cancel or select something else, then back out

                //request basepoint
                PromptPointOptions ppo = new PromptPointOptions("\nSelect basepoint.");
                //request points for segment
                Point3d first = new Point3d();
                PromptPointResult ppr = ed.GetPoint(ppo);
                if (ppr.Status == PromptStatus.OK)
                { first = ppr.Value; }
                else
                { return; }

                //request other polyline
                peo.Message = "Please choose another polyline to referance.";
                Polyline pLineRef;
                //get the polyline
                per = ed.GetEntity(peo);
                if (per.Status == PromptStatus.OK)
                { pLineRef = tr.GetObject(per.ObjectId, OpenMode.ForRead) as Polyline; }
                else
                    return;

                //request basepoint
                Point3d firstRef = new Point3d();
                ppo.Message = "\nSelect basepoint to referance.";
                ppr = ed.GetPoint(ppo);
                if (ppr.Status == PromptStatus.OK)
                { firstRef = ppr.Value; }
                else
                { return; }

                //request endpoint
                Point3d secondRef = new Point3d();
                ppo.Message = "\nSelect endpoint to referance.";
                ppr = ed.GetPoint(ppo);
                if (ppr.Status == PromptStatus.OK)
                { secondRef = ppr.Value; }
                else
                { return; }

                //specify CW or CCW
                bool CW = true;
                PromptKeywordOptions pko = new PromptKeywordOptions("\nClockWise or Counter ClockWise from basepoint?");
                pko.Keywords.Add("CW");
                pko.Keywords.Add("CCW");
                PromptResult pRes = ed.GetKeywords(pko);
                if (pRes.Status == PromptStatus.OK)
                {
                    if (pRes.StringResult.ToUpper() == "CW")
                    { CW = true; }
                    else if (pRes.StringResult.ToUpper() == "CCW")
                    { CW = false; }
                    else
                        return;
                }
                else
                    return;

                //calculate distance from two points along second polyline
                double totalDistance = findDistance(firstRef, secondRef, pLineRef);

                //calculate point along first polyline from basepoint using distance
                Point3d insertPoint = findWalkPoint(first, pLine, true, totalDistance);

                //insert notch at point
                blockInserter(bt, db, insertPoint);

                tr.Commit();
            }
        }

        //currently returns the shortest distance between the two points
        //could use the CW and CCW data to display which dist is which direction
        public double findDistance(Point3d anchor, Point3d nextPoint, Polyline pLine)
        {
            double distance = 0;
            //first find the part of the polyline that this anchor point is on
            int startPart = findPart(anchor, pLine);
            // -1 is returned if point isn't found
            if (startPart == -1)
            { return 0; }

            int lastPart = findPart(nextPoint, pLine);
            bool cwPline = clockwise(pLine);
            bool polySeg;

            //if the points are on the same part we need to check distance
            #region same part
            if (lastPart == startPart)
            {
                Curve3d seg = null;
                SegmentType segType = pLine.GetSegmentType(lastPart);
                if (segType == SegmentType.Line)
                {
                    seg = pLine.GetLineSegmentAt(lastPart);
                    LineSegment3d lDat = pLine.GetLineSegmentAt(lastPart);
                    //dist form
                    distance = anchor.DistanceTo(nextPoint);
                }
                else
                {
                    seg = pLine.GetArcSegmentAt(lastPart);
                    CircularArc3d aDat = pLine.GetArcSegmentAt(lastPart);
                    double r = aDat.Radius;
                    double liDist = anchor.DistanceTo(nextPoint);
                    distance = (.5 * liDist) / r;
                    distance = r * 2 * Math.Asin(distance);
                }



                //descern which points are closet to which end
                Point3d vertexPnt = pLine.GetPoint3dAt(lastPart);
                double anchorToStart = anchor.DistanceTo(seg.StartPoint);
                double nextToStart = nextPoint.DistanceTo(seg.StartPoint);
                if (vertexPnt == seg.StartPoint)
                {
                    if (anchorToStart < nextToStart)
                    {
                        //then anchor point is closer to start point which is the vertex we're after
                        //if we already know its a a Clockwise motion then this means the
                        //line we just made was a Clockwise path
                        if (cwPline == true)
                        { polySeg = true; }
                        else
                        { polySeg = false; }
                    }
                    else
                    {
                        //then the next point is closer to the start point
                        //if the polyline is clockwise then we are moving  counterclockwise with this line
                        if (cwPline == true)
                        { polySeg = false; }
                        else
                        { polySeg = true; }
                    }
                }

                //by designating the  point to reference the vertex # we can decide
                //if the movement was ascending or descending for CW or CCW

            #endregion
            }
            //if the points are on seperate parts
            else
            {
                //use knowledge of CW and CCW to know ahead of time which direction we're going
                double dist1 = longDistance(anchor, nextPoint, pLine, startPart, lastPart, 1);
                double dist2 = longDistance(anchor, nextPoint, pLine, startPart, lastPart, -1);
                //we'll use the shorter of the two distances <--??

                if (dist1 > dist2)
                {
                    distance = dist2;
                    if (cwPline == true)
                    { polySeg = false; }
                    else
                        polySeg = true;
                }
                else
                {
                    distance = dist1;
                    if (cwPline == true)
                    { polySeg = true; }
                    else
                        polySeg = false;
                }
                //ascending and descending should indicate ClockWise or Counter-ClockWise for the two distances
            }

            //default return, similar to null
            return distance;
        }

        //pass it the polyline, the basepoint, the targetpoint, and direction(ascending or descending)
        //should return the distance between those points along that path
        public double longDistance(Point3d basePoint, Point3d pickedPoint, Polyline pLine, int startPart, int lastPart, int direction)
        {
            double distance = 0;
            SegmentType segType = new SegmentType();

            //if direction is positive then we are moving down the vertecies, and need the endpoint
            //if the direction is negative we are moving backward along the verticies and need the startpoint
            //add in first segment
            distance = segmentDist(basePoint, pLine, startPart, direction);
            //add in last segment
            distance = distance + segmentDist(pickedPoint, pLine, lastPart, direction * -1);

            startPart = startPart + direction;

            //loop through any segments which are totally used and do not have the last point
            for (int i = startPart; i != lastPart; i = i + direction)
            {
                //make sure index doenst become a vertex which doesnt exist
                if (i == -1)
                {
                    i = pLine.NumberOfVertices - 1;
                    if (lastPart == pLine.NumberOfVertices - 1)
                    { return distance; }
                }
                else if (i == pLine.NumberOfVertices)
                {
                    i = 0;
                    if (lastPart == 0)
                    { return distance; }
                }

                segType = pLine.GetSegmentType(i);
                if (segType == SegmentType.Line)
                {
                    LineSegment3d lseg = pLine.GetLineSegmentAt(i);
                    distance = distance + lseg.Length;
                }
                else
                {
                    CircularArc3d aSeg = pLine.GetArcSegmentAt(i);
                    double r = aSeg.Radius;
                    double liDist = aSeg.StartPoint.DistanceTo(aSeg.EndPoint);
                    double dist = (.5 * liDist) / r;
                    dist = r * 2 * Math.Asin(dist);

                    distance = distance + dist;
                }
            }

            //return that distance
            return distance;
        }

        public double segmentDist(Point3d p, Polyline pLine, int partIndex, int direction)
        {
            Curve3d seg = null;
            double distance = 0;

            //if direction is positive then we are moving down the vertecies, and need the endpoint
            //if the direction is negative we are moving backward along the verticies and need the startpoint
            SegmentType segType = pLine.GetSegmentType(partIndex);
            if (segType == SegmentType.Line)
            {
                seg = pLine.GetLineSegmentAt(partIndex);
                if (direction > 0)
                    distance = seg.EndPoint.DistanceTo(p);
                else
                    distance = seg.StartPoint.DistanceTo(p);
            }
            else
            {
                seg = pLine.GetArcSegmentAt(partIndex);

                Point3d nextPoint = new Point3d();
                if (direction > 0)
                { nextPoint = seg.EndPoint; }
                else
                { nextPoint = seg.StartPoint; }

                CircularArc3d aDat = pLine.GetArcSegmentAt(partIndex);
                double r = aDat.Radius;
                double liDist = p.DistanceTo(nextPoint);
                if (liDist == 0)
                {
                    return 0;
                }
                distance = (.5 * liDist) / r;
                distance = r * 2 * Math.Asin(distance);
            }

            return distance;
        }

        public int findPart(Point3d start, Polyline pLine)
        {
            int index = -1;
            Curve3d seg = null;
            //loop through the poly line segment by segment
            for (int i = 0; i < pLine.NumberOfVertices; i++)
            {
                SegmentType segType = pLine.GetSegmentType(i);
                if (segType == SegmentType.Line)
                { seg = pLine.GetLineSegmentAt(i); }
                else
                { seg = pLine.GetArcSegmentAt(i); }

                if (seg != null)
                {
                    if (seg.IsOn(start) == true)
                        return i;
                }
            }

            return index;
        }

        public bool clockwise(Polyline pline)
        {
            bool CW = true;
            //for each vertex in the poly line
            //calculate a cross product to decide if it is pos or neg
            //if more vertices in a polygon are pos it is CCW
            //if more vertices in a polygon are neg it is CW
            int pos = 0;
            int neg = 0;

            for (int i = 0; i < pline.NumberOfVertices; i++)
            {
                double answer;
                //change the algorythm a touch for the first and last vertex
                if (i == 0)
                {
                    //pass the previous vertex, current vertex, and next vertex
                    answer = crossProduct(pline.GetPoint3dAt(pline.NumberOfVertices - 1), pline.GetPoint3dAt(i), pline.GetPoint3dAt(i + 1));
                }
                else if (i == (pline.NumberOfVertices - 1))
                { answer = crossProduct(pline.GetPoint3dAt(i - 1), pline.GetPoint3dAt(i), pline.GetPoint3dAt(0)); }
                else
                { answer = crossProduct(pline.GetPoint3dAt(i - 1), pline.GetPoint3dAt(i), pline.GetPoint3dAt(i + 1)); }

                if (answer > 0)
                { pos++; }
                else if (answer < 0)
                { neg++; }
            }

            if (pos > neg)
            { CW = false; }
            else
            { CW = true; }

            return CW;
        }

        //pass the points of the vertex we're examining and its preceding, proceding points
        //cross multiply to find the area and if protrudes pos or neg from the plane
        public double crossProduct(Point3d v1, Point3d v2, Point3d v3)
        {
            double answer = ((v2.X - v1.X) * (v3.Y - v2.Y)) - ((v2.Y - v1.Y) * (v3.X - v1.X));
            return answer;
        }

        public Point3d findWalkPoint(Point3d basePoint, Polyline pLine,
        bool CW,
        double distance)
        {
            bool plineCw = clockwise(pLine);
            int direction = 0;
            //the Clockwise bool being passed in is for the direction we want our line to go
            //if the second poly line is clockwise then it directly corallates to the line seg
            //if the second poly line is CCW then it inversely corallates to the line segment
            if (plineCw == true)
            {
                if (CW == true)
                { direction = 1; }
                else
                { direction = -1; }
            }
            else
            {
                if (CW == true)
                { direction = -1; }
                else
                { direction = 1; }
            }

            //find the part the basePoint is on
            int startPart = findPart(basePoint, pLine);
            //use direction to decide if ascending or desceding
            //get first segment dist from base point to start or end *based on direction
            double runningDist = segmentDist(basePoint, pLine, startPart, direction);
            SegmentType segType = new SegmentType();
            int index = startPart;
            Curve3d seg = null;

            //if base point isnt on a vertex, need to get the piece of the segment first, then add it accordingly

            //add the following segments until distance is greater than distance being sought
            while (runningDist < distance)
            {
                index = index + direction;
                //make sure the index doesnt become an out of bounds vertex
                if (index == -1)
                { index = pLine.NumberOfVertices - 1; }
                else if (index == pLine.NumberOfVertices)
                { index = 0; }

                //get the segment of each piece and add it on
                segType = pLine.GetSegmentType(index);
                if (segType == SegmentType.Line)
                {
                    LineSegment3d lseg = pLine.GetLineSegmentAt(index);
                    runningDist = runningDist + lseg.Length;
                }
                else
                {
                    CircularArc3d aSeg = pLine.GetArcSegmentAt(index);
                    double r = aSeg.Radius;
                    double liDist = aSeg.StartPoint.DistanceTo(aSeg.EndPoint);
                    double dist = (.5 * liDist) / r;
                    dist = r * 2 * Math.Asin(dist);

                    runningDist = runningDist + dist;
                }
            }

            //get the segment used last
            segType = pLine.GetSegmentType(index);
            if (segType == SegmentType.Line)
            { seg = pLine.GetLineSegmentAt(index); }
            else
            { seg = pLine.GetArcSegmentAt(index); }

            //determine the last point in segment that both the distance and running distance would have
            Point3d sharedPoint = new Point3d();
            Point3d lastPoint = new Point3d();
            //depends on direction
            if (direction > 0)
            {
                sharedPoint = seg.StartPoint;
                lastPoint = seg.EndPoint;
            }
            else if (direction < 0)
            {
                sharedPoint = seg.EndPoint;
                lastPoint = seg.StartPoint;
            }

            //find the point on the last segment using the excess distance between the two
            double remainder = Math.Abs(runningDist - distance);
            Point3d finalPoint = new Point3d();
            //if remainder is 0 then it was on an endpoint we ended
            if (remainder == 0)
            {
                finalPoint = lastPoint;
                return lastPoint;
            }
            //otherwise need to get the last segment distance and find the point as it would fall               
            else
            {
                //back up from the the endpoint of the total distance of the last segment the distance of the remainder
                if (segType == SegmentType.Line)
                {
                    LineSegment3d lDat = pLine.GetLineSegmentAt(index);
                    List<Point3d> points = lineCircIntersect(lDat, remainder, lastPoint);
                    //verify that the points are on the segment
                    points = isPointOnCurve(points, pLine);
                    //determine which point is closer to the previous point (shared Point)
                    finalPoint = findClosestPoint(points, sharedPoint);

                    return finalPoint;
                }
                else
                {
                    CircularArc3d aDat = pLine.GetArcSegmentAt(index);
                    List<Point3d> points = notchOnArc(aDat, remainder, lastPoint);
                    //verify that the points are on the segment
                    points = isPointOnCurve(points, pLine);
                    //determine which point is closer to the previous point
                    finalPoint = findClosestPoint(points, sharedPoint);
                    return finalPoint;
                }
            }

            //this point will be used to calculate slopes *for rotation and rotation point
        }

        //calculates intersections between a line and circle
        public List<Point3d> lineCircIntersect(LineSegment3d lDat, double r, Point3d center)
        {
            //find known constants
            double m = getSlopeLine(lDat);
            double p = center.X;
            double q = center.Y;
            double c = findYint(lDat.StartPoint, m);

            List<Point3d> points = new List<Point3d>();
            Point3d first;
            Point3d second;

            //if slope is 0 then it is horizontal
            if (m == 0)
            {
                first = new Point3d(center.X + r, center.Y, center.Z);
                second = new Point3d(center.X - r, center.Y, center.Z);
            }
            //if slope is an infinity value it is vertical
            else if (m == double.PositiveInfinity || m == double.NegativeInfinity)
            {
                first = new Point3d(center.X, center.Y + r, center.Z);
                second = new Point3d(center.X, center.Y - r, center.Z);
            }
            else
            {
                //using line equation  y=mx+c
                //using circle equation (x-p)^2 + (y-q)^2 =r^2   (p,h) = center
                //----------------
                // (x-p)^2 + (mx+c-q)^2 = r^2
                // (m^2 +1)x^2 = 2(mc - mq-p)x + (q^2 - r^2 + p^2 - 2cq + c^2) = 0
                //     A              B                         C
                //-----
                //quadratic formula  x= (-B +/- (B2 - 4AC)^.5) / 2A
                // plug back into line
                //   y= m ((-B +/- (B2 - 4AC)^.5) / 2A) + c
                // plug A,B,C back into the formula

                double quadNumerator = -2 * (m * c - m * q - p) + Math.Sqrt(Math.Pow((2 * (m * c - m * q - p)), 2) - 4 * (m * m + 1) * (q * q - r * r + p * p - 2 * c * q + c * c));
                double quadNumNeg = -2 * (m * c - m * q - p) - Math.Sqrt(Math.Pow((2 * (m * c - m * q - p)), 2) - 4 * (m * m + 1) * (q * q - r * r + p * p - 2 * c * q + c * c));
                double quadDenom = 2 * (m * m + 1);
                double QuadForm = quadNumerator / quadDenom;
                double QuadFormNeg = quadNumNeg / quadDenom;
                double y = m * QuadForm + c;
                double yNeg = m * QuadFormNeg + c;

                //use the +/- results to solve for X
                double x = (y - c) / m;
                double xNeg = (yNeg - c) / m;

                first = new Point3d(x, y, 0);
                second = new Point3d(xNeg, yNeg, 0);
            }

            //return the two points            
            points.Add(first);
            points.Add(second);

            return points;
        }

        public List<Point3d> notchOnArc(CircularArc3d aDat, double remainder, Point3d sharedPoint)
        {
            List<Point3d> points = new List<Point3d>();

            Point3d insertPoint1 = new Point3d();
            Point3d insertPoint2 = new Point3d();

            Point3d center = aDat.Center;
            double alpha = remainder / aDat.Radius;
            double beta = Math.Acos(((sharedPoint.X - aDat.Center.X) / aDat.Radius));

            //grabbing the angle from CAD ignored which point we were measuring from
            //if (aDat.StartAngle > aDat.EndAngle)
            //{ beta = aDat.StartAngle; }
            //else
            //{ beta = aDat.EndAngle; }

            double theta = beta - alpha;
            double xDim = aDat.Radius * Math.Round(Math.Cos(theta), 9);
            double yDim = aDat.Radius * Math.Round(Math.Sin(theta), 9);

            double xDimAlt = aDat.Radius * Math.Round(Math.Cos(beta + alpha), 9);
            double yDimAlt = aDat.Radius * Math.Round(Math.Sin(beta + alpha), 9);

            insertPoint1 = new Point3d(center.X + xDim, center.Y + yDim, 0);
            insertPoint2 = new Point3d(center.X + xDimAlt, center.Y + yDimAlt, 0);

            points.Add(insertPoint1);
            points.Add(insertPoint2);

            return points;
        }

        //get the y intersect
        private double findYint(Point3d point, double m)
        {
            double yIntercept = point.Y - m * point.X;
            return yIntercept;
        }

        private double getSlopeLine(LineSegment3d lDat)
        {
            double x1, x2, y1, y2;
            x2 = Math.Round(lDat.EndPoint.X, 7);
            x1 = Math.Round(lDat.StartPoint.X, 7);
            y2 = Math.Round(lDat.EndPoint.Y, 7);
            y1 = Math.Round(lDat.StartPoint.Y, 7);
            //double m = (Math.Round(lDat.EndPoint.Y, 9) - Math.Round(lDat.StartPoint.Y,9)) / (Math.Round(lDat.EndPoint.X,9) - Math.Round(lDat.StartPoint.Y,9));
            //need to be sure to handle infinties and 0 slopes
            double m = (y2 - y1) / (x2 - x1);
            return m;
        }

        //flaw to this is that its possible that both points exist on the polyline,
        //needs to look specifically at just the part segment, not whole polyline
        //might simply have it loop through the poly line and make DBObjects that
        //can be used to compare to the endpoint,startpoint of the seg data
        //if one matches then it gets passed and used for a Curve
        public List<Point3d> isPointOnCurve(List<Point3d> points, Polyline pLine)
        {
            List<Point3d> returnPoints = new List<Point3d>();
            foreach (Point3d p in points)
            {
                try
                {
                    Point3d pt = pLine.GetClosestPointTo(p, false);
                    double d = (pt - p).Length;
                    if (d < 0.001)
                    { returnPoints.Add(p); }
                }
                catch { }
            }

            return returnPoints;
        }

        //hand over the two intersection points, calculate distance from the previous endPoint
        //whichever is the closest point gets returned
        public Point3d findClosestPoint(List<Point3d> points, Point3d endPoint)
        {
            double distance = 0;
            Point3d returnMe = new Point3d();
            foreach (Point3d p in points)
            {
                double d = Math.Sqrt((Math.Pow((p.X - endPoint.X), 2) + Math.Pow((p.Y - endPoint.Y), 2)));
                if (d < distance || distance == 0)
                {
                    returnMe = p;
                    distance = d;
                }
            }
            return returnMe;
        }

        //inserts the notch where we asked
        private static void blockInserter(BlockTable bt, Database db, Point3d insertPoint)
        {
            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                Document doc = Application.DocumentManager.MdiActiveDocument;
                Editor ed = doc.Editor;

                string notchPath = @"Y:\Product Development\AutoCAD LT\Blocks\Fabric Layout\FL_Notch.dwg";
                string specName = Path.GetFileNameWithoutExtension(notchPath);
                ObjectId blkReciD = ObjectId.Null;
                if (!bt.Has(specName))
                {
                    //open the other DB
                    Database dbImport = new Database(false, true);
                    try
                    { dbImport.ReadDwgFile(notchPath, FileShare.Read, false, ""); }
                    catch (System.Exception)
                    {
                        ed.WriteMessage("/nUnable to read drawing file.");
                        return;
                    }
                    db.Insert(specName, dbImport, true);

                    blkReciD = bt[specName];
                }
                else
                { blkReciD = bt[specName]; }

                //now insert block into current space
                if (blkReciD != ObjectId.Null)
                {
                    //open a new btr for the blkrec we're working with
                    BlockTableRecord btrInsert = tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite) as BlockTableRecord;
                    using (BlockReference blkRef = new BlockReference(insertPoint, blkReciD))
                    {
                        btrInsert.AppendEntity(blkRef);
                        tr.AddNewlyCreatedDBObject(blkRef, true);
                        tr.TransactionManager.QueueForGraphicsFlush();
                    }
                }
                tr.Commit();
            }
        }
    }
}
