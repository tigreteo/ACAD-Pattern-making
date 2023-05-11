using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Distance
{
    //user selects a polyline (hopefully works for closed and open
    //user then selects two points
    //results in the distance between the points (distances for closed loops)
    public class Class1
    {
        //command requests an entity(polyline) and two points
        //find the distance(s) between points using verteces and segments
        [CommandMethod("PerimeterDist")]
        public void basicPerim()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;

            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                //request the polyline
                //filter for only polylines
                PromptEntityOptions peo = new PromptEntityOptions("\nPlease choose a polyine.");
                peo.SetRejectMessage("\nPlease be sure to click a polyline.");
                peo.AddAllowedClass(typeof(Polyline),true);

                Polyline pLine;
                //get the polyline
                PromptEntityResult per = ed.GetEntity(peo);
                if (per.Status == PromptStatus.OK)
                { pLine = tr.GetObject(per.ObjectId, OpenMode.ForRead) as Polyline; }
                else
                    return;//if they cancel or select something else, then back out

                PromptPointOptions ppo = new PromptPointOptions("\nChoose a segment start Point.");
                //request points for segment
                Point3d first = new Point3d();
                PromptPointResult ppr = ed.GetPoint(ppo);
                if(ppr.Status == PromptStatus.OK)
                { first = ppr.Value; }
                else
                { return; }

                //get second point
                Point3d second = new Point3d();
                ppo.Message = "\nChoose the segment end Point";
                ppr = ed.GetPoint(ppo);
                if(ppr.Status == PromptStatus.OK)
                { second = ppr.Value; }
                else
                { return; }


                double distance = findDistance(first, second, pLine);
                ed.WriteMessage("\n Shortest Distance: " + distance);
                tr.Commit();
            }
        }


        //choose a poly line and two points
        //offsets the polyline the seam distance, default .5, calculates the distance that way
        [CommandMethod("SeamDistance")]
        public void seamDist()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;

            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                BlockTable bt = tr.GetObject(db.BlockTableId, OpenMode.ForWrite) as BlockTable;
                BlockTableRecord btr = tr.GetObject(bt[BlockTableRecord.ModelSpace], OpenMode.ForWrite) as BlockTableRecord;

                //request the polyline
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

                PromptPointOptions ppo = new PromptPointOptions("\nChoose a segment start Point.");
                //request points for segment
                Point3d first = new Point3d();
                PromptPointResult ppr = ed.GetPoint(ppo);
                if (ppr.Status == PromptStatus.OK)
                { first = ppr.Value; }
                else
                { return; }

                //get second point
                Point3d second = new Point3d();
                ppo.Message = "\nChoose the segment end Point";
                ppr = ed.GetPoint(ppo);
                if (ppr.Status == PromptStatus.OK)
                { second = ppr.Value; }
                else
                { return; }

                //get seam dist
                double seam;
                PromptDoubleOptions pdo = new PromptDoubleOptions("\nSeam distance: ");
                pdo.DefaultValue = .5;
                PromptDoubleResult pdr = ed.GetDouble(pdo);
                if (pdr.Status == PromptStatus.OK)
                { seam = Math.Abs(pdr.Value); }
                else
                    return;

                //must set the seam dist to negative to offset inward
                DBObjectCollection dboCol = pLine.GetOffsetCurves(seam * -1);

                Polyline offsetPline = new Polyline();
                //take the dbocollection and turn it into a polyline to work with
                if (dboCol.Count > 0)
                { offsetPline = dboCol[0] as Polyline; }

                //find the nearest points on the new polyline from the original poins (i.e. perpendicular points)
                Point3d offFirst = offsetPline.GetClosestPointTo(first, true);
                Point3d offSecond = offsetPline.GetClosestPointTo(second, true);

                double distance = findDistance(offFirst, offSecond, offsetPline);
                ed.WriteMessage("\n Shortest Distance: " + distance);

                tr.Commit();
            }
        }

        //using the parts of the object collection it will find shared vertices and
        //order them into a poly line
        //if a piece is an arc, it will change the vertex before it to have
        //appropriate bulge
        private static Polyline drawPolyLine2(DBObjectCollection dbObjCol, BlockTableRecord btr, Transaction tr)
        {
            //iterate through the collection adding parts based on if they connect to the previous part
            Polyline pLine = new Polyline();
            //load the first piece of the poly line into the Poly Line
            DBObject lastPart = dbObjCol[0];
            Point3d endPoint = new Point3d();
            Point3d startPoint = new Point3d();
            bool firstArc = false;
            double bulgeHold = new double();

            if (lastPart is Arc)
            {
                Arc a = lastPart as Arc;
                endPoint = a.EndPoint;
                startPoint = a.StartPoint;
            }
            else if (lastPart is Line)
            {
                Line l = lastPart as Line;
                endPoint = l.EndPoint;
                startPoint = l.StartPoint;
            }

            //iterate through the collection to get all vertices
            for (int i = 0; i < dbObjCol.Count; i++)
            {
                //use method to find the next part that touches the previous vertex
                DBObject dbObj = findNextPart(dbObjCol, lastPart, endPoint, startPoint);

                Point3d targetPoint = new Point3d();
                //be sure to use the point that WASNT the last point used
                //as the new vertex
                if (dbObj is Arc)
                {
                    Arc arcDat = dbObj as Arc;
                    Point3d a = arcDat.StartPoint;
                    Point3d b = arcDat.EndPoint;
                    double bulge;
                    if (a == endPoint)
                    {
                        targetPoint = b;
                        bulge = getArcBulge(arcDat.EndAngle, arcDat.StartAngle);

                    }
                    else
                    {
                        targetPoint = a;
                        bulge = getArcBulge(arcDat.EndAngle, arcDat.StartAngle);
                        bulge = 0 - bulge;
                    }

                    pLine.AddVertexAt(i, new Point2d(targetPoint.X, targetPoint.Y),
                        0, 0, 0);
                    //if the first part is an arc, we'll need to hold onto the bulge until the end
                    if (i > 0)
                    { pLine.SetBulgeAt(i - 1, bulge); }
                    else
                    {
                        firstArc = true;
                        bulgeHold = bulge;
                    }

                    pLine.Layer = arcDat.Layer;
                    pLine.Color = arcDat.Color;

                    //load the next parts in
                    startPoint = endPoint;
                    endPoint = targetPoint;

                }
                else if (dbObj is Line)
                {
                    Line lDat = dbObj as Line;
                    Point3d a = lDat.StartPoint;
                    Point3d b = lDat.EndPoint;
                    if (a == endPoint)
                    { targetPoint = b; }
                    else
                    { targetPoint = a; }

                    pLine.AddVertexAt(i, new Point2d(targetPoint.X, targetPoint.Y),
                        0, 0, 0);
                    pLine.Layer = lDat.Layer;
                    pLine.Color = lDat.Color;
                    //load up the next parts' points
                    startPoint = endPoint;
                    endPoint = targetPoint;
                }

                lastPart = dbObj;
            }

            if (firstArc == true)
            { pLine.SetBulgeAt(dbObjCol.Count - 1, bulgeHold); }

            //make the new pLine an obvious color for now
            pLine.ColorIndex = 0;

            //add to the btr and show
            pLine.Closed = true;
            btr.AppendEntity(pLine);
            tr.TransactionManager.AddNewlyCreatedDBObject(pLine, true);
            return pLine;
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

        //iterates through a collection of segments from an exploded polyline
        //to find which part connects to the one listed
        private static DBObject findNextPart(DBObjectCollection dbObjCol, DBObject dbObj, Point3d sharedPoint, Point3d oldPoint)
        {
            Point3d startPoint = new Point3d();
            Point3d endPoint = new Point3d();
            foreach (DBObject part in dbObjCol)
            {
                if (part != dbObj)
                {
                    //get the endPoint/start points
                    if (part is Arc)
                    {
                        Arc a = part as Arc;
                        startPoint = a.StartPoint;
                        endPoint = a.EndPoint;
                    }
                    if (part is Line)
                    {
                        Line l = part as Line;
                        startPoint = l.StartPoint;
                        endPoint = l.EndPoint;
                    }

                    if (sharedPoint == startPoint && endPoint != oldPoint)
                        return part;
                    else if (sharedPoint == endPoint && startPoint != oldPoint)
                        return part;
                }
            }
            return null;
        }

        //pass a segment, based on the slope at that point on that seg, it will use the appropriate slope algorythm
        private double slopeAtPoint(Point3d point, Polyline pLine, int index)
        {
            double m = new double();
            SegmentType segType = pLine.GetSegmentType(index);
            if (segType == SegmentType.Line)
            {
                LineSegment3d lDat = pLine.GetLineSegmentAt(index);
                m = getSlopeLine(lDat);
            }
            else
            {
                CircularArc3d aDat = pLine.GetArcSegmentAt(index);
                m = getSlopeArc(aDat, point);
            }
            return m;
        }

        private double getSlopeArc(CircularArc3d aDat, Point3d tpoint)
        {
            double slope = 0;
            double x1, x2, y1, y2;
            x2 = Math.Round(aDat.Center.X, 7);
            x1 = Math.Round(tpoint.X);
            y2 = Math.Round(aDat.Center.Y, 7);
            y1 = Math.Round(tpoint.Y, 7);

            //get slope of the radius as it touches the endpoint
            //double m = (aDat.Center.Y - tpoint.Y) / (aDat.Center.X - tpoint.X);
            double m = (y2 - y1) / (x2 - x1);

            //don't know if it has to account for undefined numbers here, code seems to account for it later
            if (m == double.NegativeInfinity | m == double.PositiveInfinity)
            { return Math.PI; }
            else if (m == 0)
            { return Math.PI / 2; }
            else
                slope = -1 * (1 / m);

            //be sure to check for infinities and 0 slopes
            return slope;
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

        //calculates intersections between a line and circle
        public List<Point3d> lineCircIntersect(double m, double r, Point3d center)
        {
            //find known constants
            double p = center.X;
            double q = center.Y;
            double c = findYint(center, m);

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

        //get the y intersect
        private double findYint(Point3d point, double m)
        {
            double yIntercept = point.Y - m * point.X;
            return yIntercept;
        }

        //flaw to this is that its possible that both points exist on the polyline,
        //needs to look specifically at just the part segment, not whole polyline
        //might simply have it loop through the poly line and make DBObjects that
        //can be used to compare to the endpoint,startpoint of the seg data
        //if one matches then it gets passed and used for a Curve
        public Point3d isPointOnCurve(List<Point3d> points, Polyline pLine)
        {
            //List<Point3d> returnPoints = new List<Point3d>();
            foreach (Point3d p in points)
            {
                try
                {
                    Point3d pt = pLine.GetClosestPointTo(p, true);
                    double d = (pt - p).Length;
                    if (d < 0.001)
                    { return pt; }
                }
                catch { }
            }
            //default if none of them are on the correct line
            return new Point3d(0, 0, 0);
        }

        //using a formula I found in forums. find what CAD considers the bulge of the arc
        //bulge = tan(included angle/4)
        //positive is counterclockwise
        private static double getArcBulge(double endAngle, double startAngle)
        {
            double deltaAng = endAngle - startAngle;
            if (deltaAng < 0)
                deltaAng += 2 * Math.PI;

            return Math.Tan(deltaAng * .25);
        }
    }
}
