using Autodesk.AutoCAD.Runtime;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.EditorInput;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace Notcher
{
    //Class used to make notching patterns easier
    //request a distance to add notches
    //splits the distance and adds them centered to chosen point
    //  following the direction of the polyline
    //  Will not work properly on verteces
    //  Currently intended to work on just centerpoints

    public class Class1
    {    
        [CommandMethod("Notcher")]
        public void Start()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;
            
            //request a distance for a the notches
            double distance = 0;
            PromptDoubleOptions sto = new PromptDoubleOptions("\nEnter distance between notches: ");
            PromptDoubleResult pRes = ed.GetDouble(sto);            
            if (pRes.Status == PromptStatus.OK)
            { distance = pRes.Value; }
            //request a point for on a polyline

            PromptPointResult pntRes;
            PromptPointOptions pOpts = new PromptPointOptions("");
            pOpts.Message = "\nCenter point of notches: ";

            Point3d startPoint = new Point3d();
            pntRes = ed.GetPoint(pOpts);
            if (pntRes.Status == PromptStatus.OK)
            { startPoint = pntRes.Value;}

            //account for canceling
            if (pntRes.Status == PromptStatus.Cancel)
                return;

            //use point to select the poly line...
            SelectionSet targetPart = null;
            PromptSelectionResult ssPrompt = ed.SelectCrossingWindow(
                new Point3d(startPoint.X - .001, startPoint.Y - .001, 0),
                new Point3d(startPoint.X + .001, startPoint.Y + .001, 0));

            if (ssPrompt.Status == PromptStatus.OK)
            { targetPart = ssPrompt.Value; }

            //start a transaction
            using (Transaction tr = db.TransactionManager.StartTransaction() as Transaction)
            {
                if (targetPart.Count > 0)
                {
                    foreach (ObjectId objId in targetPart.GetObjectIds())
                    {
                        Entity ent = tr.GetObject(objId, OpenMode.ForRead) as Entity;
                        if (ent is Polyline)
                        {
                            //pull the poly line from the selection set
                            //most likely only had the polyline and maybe a blockRef
                            Polyline pLine = ent as Polyline;
                            drawNotch(doc, startPoint, pLine, db, distance, tr);
                        }
                    }
                }
                tr.Commit();
            }
        }

        //use the selected point and selected pattern to proceed
        private static void drawNotch(Document doc, Point3d startPoint, Polyline pLine, Database db, double goalDist, Transaction tr)
        {                         
            BlockTable bt = tr.GetObject(db.BlockTableId, OpenMode.ForRead) as BlockTable;
            BlockTableRecord btr = tr.GetObject(bt[BlockTableRecord.ModelSpace], OpenMode.ForRead) as BlockTableRecord;

            //explode the polyline into a collection
            DBObjectCollection pattern = new DBObjectCollection();
            pLine.Explode(pattern);

            DBObjectCollection startPartCol = new DBObjectCollection();
            //search througth the lines/arcs to find the one that contains the start point
            foreach (DBObject obj in pattern)
            {
                if (obj is Line)
                {
                    //this doesnt just check if the point is there, but gives some wiggle room for rounding issues
                    Line dbLine = obj as Line;
                    LineSegment3d lSeg = new LineSegment3d(dbLine.StartPoint, dbLine.EndPoint);
                    PointOnCurve3d q = lSeg.GetClosestPointTo(startPoint);
                    if (startPoint.DistanceTo(q.Point) < 0.001)
                    { startPartCol.Add(obj); }
                }
                else if (obj is Arc)
                {
                    Arc dbArc = obj as Arc;
                    try
                    {
                        dbArc.GetDistAtPoint(startPoint);
                        startPartCol.Add(obj);
                    }
                    catch 
                    { }
                    //otherwise it wasn't on the arc
                }
            }

            //calculate distance from start to the endpoints of the segment the point is on
            //hopefully only one segment was found
            //CURRENTLY ONLY WORKS IF CHOSEN POINT IS IN CENTER
            //OTHERWISE THE DISTANCES ARE DIFF
            double distance = getDistance(startPartCol, startPoint);              

            goalDist = goalDist / 2;
                
            //prime the loop
            DBObject startPart = startPartCol[0];
            Point3d endPoint1 = new Point3d();
            Point3d endPoint2 = new Point3d();

            if (startPart is Line)
            {
                Line lDat = startPart as Line;
                endPoint1 = lDat.EndPoint;
                endPoint2 = lDat.StartPoint;
            }
            else if (startPart is Arc)
            {
                Arc aDat = startPart as Arc;
                endPoint1 = aDat.EndPoint;
                endPoint2 = aDat.StartPoint;
            }

            //simply reversing the points we're starting from to get the notches to go seperate directions

            //get the first notch location
            Point3d insertPoint1 = findInsertPoint(startPart, endPoint1, startPoint, goalDist, distance, pattern, pLine);
            //get the second notch loc
            Point3d insertPoint2 = findInsertPoint(startPart, endPoint2, startPoint, goalDist, distance, pattern, pLine);
            //insert the notch at the point now
            blockInserter(bt, db, insertPoint1);
            blockInserter(bt, db, insertPoint2);
            //rotate the  notch to be perpendicular to the line/arc segment it resides on            
        }

        //simply uses the angle given by CAD, and the distance left to cover and 
        //uses sine and cosine to solve the X and Y offsets from the end point
        private static Point3d findInsertPoint(DBObject startPart, Point3d sharedPoint,Point3d lastPoint, double goalDist, double distance, DBObjectCollection pattern, Polyline pLine)
        {
            //if the distance is not that of the requested distance(divided by 2)
            //  proceed to add the following segments until the distance added is greater than
            //  the requested distance half
            while (goalDist > distance)
            {
                //find next part connected to current part
                startPart = findNextPart(pattern, startPart, sharedPoint, lastPoint);

                //distance = next part                       
                if (startPart is Line)
                {
                    Line lDat = startPart as Line;
                    distance = distance + lDat.Length;
                    if (lDat.StartPoint == sharedPoint)
                    {
                        sharedPoint = lDat.EndPoint;
                        lastPoint = lDat.StartPoint;
                    }
                    else
                    {
                        sharedPoint = lDat.StartPoint;
                        lastPoint = lDat.EndPoint;
                    }
                }
                else if (startPart is Arc)
                {
                    Arc aDat = startPart as Arc;
                    distance = distance + aDat.Length;

                    if (aDat.StartPoint == sharedPoint)
                    {
                        sharedPoint = aDat.EndPoint;
                        lastPoint = aDat.StartPoint;
                    }
                    else
                    {
                        sharedPoint = aDat.StartPoint;
                        lastPoint = aDat.EndPoint;
                    }
                }
            }

            Point3d insertPoint = new Point3d();
            //need to pull the algorythms into seperate methods for the arc vs line finding a point


            //  subtract the fist distance and added distance from the orginal
            double remainder = goalDist - distance;
            remainder = Math.Abs(remainder);
            //  use the remaning distance to calculate where this would end on this segment
            //  back up from the end of the last segment the distance of the remainder (sharedpoint toward endpoint)
            if (startPart is Line)
            {
                Line lDat = startPart as Line;
                List<Point3d> points = lineCircIntersect(lDat, remainder, sharedPoint);
                //verify that the points are on the segment
                points = isPointOnCurve(points, startPart);
                //determine which point is closer to the previous point
                insertPoint = findClosestPoint(points, lastPoint);
            }
            else if (startPart is Arc)
            {
                Arc aDat = startPart as Arc;
                List<Point3d> points = notchOnArc(aDat, remainder, sharedPoint);
                //verify that the points are on the segment
                points = isPointOnCurve(points, startPart);
                //determine which point is closer to the previous point
                insertPoint = findClosestPoint(points, lastPoint);
            }
            return insertPoint;
        }

        private static List<Point3d> isPointOnCurve(List<Point3d> points, DBObject part)
        {
            List<Point3d> returnPoints = new List<Point3d>();
            Curve cv = part as Curve;
            foreach(Point3d p in points)
            {
                try
                {
                    Point3d pt = cv.GetClosestPointTo(p, false);
                    double d = (pt - p).Length;
                    if (d < 0.001)
                    { returnPoints.Add(p); }
                }
                catch { }
            }

            return returnPoints;
        }

        //checks if a point is on the poly line but leaves no room for rounding errors
        //including the internal rounding found on some CAD points
        private static List<Point3d> onSegment(List<Point3d> points, DBObject part)
        {
            List<Point3d> returnPoints = new List<Point3d>();

            foreach(Point3d p in points)
            {
                if (part is Line)
                {
                    Line l = part as Line;
                    LineSegment3d lSeg = new LineSegment3d(l.StartPoint, l.EndPoint);
                    PointOnCurve3d q = lSeg.GetClosestPointTo(p);
                    if(p.DistanceTo(q.Point) < 0.001)
                    { returnPoints.Add(q.Point); }
                }
                else if (part is Arc)
                {
                    Arc a = part as Arc;
                    LineSegment3d lSeg = new LineSegment3d(a.StartPoint, a.EndPoint);
                    PointOnCurve3d q = lSeg.GetClosestPointTo(p);
                    if(p.DistanceTo(q.Point) < 0.001)
                    { returnPoints.Add(q.Point); }
                }
            }
            return returnPoints;
        }

        //checks if a point is on the poly line but leaves no room for rounding errors
        //including the internal rounding found on some CAD points
        private static  List<Point3d> onSeg(List<Point3d> points, Polyline p1)
        {
            Curve3d seg = null;
            List<Point3d> returnList =  new List<Point3d>();
            foreach(Point3d p in points)
            {
                for (int i = 0; i < p1.NumberOfVertices; i++)
                {
                    SegmentType segType = p1.GetSegmentType(i);
                    if (segType == SegmentType.Arc)
                    { seg = p1.GetArcSegmentAt(i); }
                    else if (segType == SegmentType.Line)
                    { seg = p1.GetLineSegmentAt(i); }

                    if (seg != null)
                    {
                        if (seg.IsOn(p) == true)
                        { returnList.Add(p); }
                    }
                }            
            }
            return returnList;               
        }

        //hand over the two intersection points, calculate distance from the previous endPoint
        //whichever is the closest point gets returned
        private static Point3d findClosestPoint (List<Point3d> points, Point3d endPoint)
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

        //calculates intersections between a line and circle
        private static List<Point3d> lineCircIntersect(Line lDat, double r, Point3d center)
        {
            //find known constants
            double m = findSlope(lDat.StartPoint, lDat.EndPoint);
            double p = center.X;
            double q = center.Y;
            double c = findYint(lDat.StartPoint, m);

            List<Point3d> points = new List<Point3d>();
            Point3d first;
            Point3d second;

            //if slope is 0 then it is horizontal
            if(m == 0)
            {
                first = new Point3d(center.X + r, center.Y, center.Z);
                second = new Point3d(center.X - r, center.Y, center.Z);
            }
            //if slope is an infinity value it is vertical
            else if (m == double.PositiveInfinity || m == double.NegativeInfinity)
            {
                first = new Point3d(center.X , center.Y + r, center.Z);
                second = new Point3d(center.X , center.Y - r, center.Z);
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

        private static List<Point3d> notchOnArc(Arc aDat, double remainder, Point3d sharedPoint)
        {
            List<Point3d> points = new List<Point3d>();

            Point3d insertPoint1 = new Point3d();
            Point3d insertPoint2 = new Point3d();

            Point3d center = aDat.Center;
            double alpha = remainder / aDat.Radius;
            double beta = Math.Acos(((sharedPoint.X - aDat.Center.X)/aDat.Radius));
            
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

        //OLD CODE couldn't account for direction of polyline
        private static Point3d notchOnLine(Line lDat, double remainder, Point3d sharedPoint)
        {
            Point3d insertPoint = new Point3d();

            double theta = lDat.Angle;
            double xDim = remainder * Math.Round(Math.Cos(theta), 9);
            double yDim = remainder * Math.Round(Math.Sin(theta), 9);

            insertPoint = new Point3d(sharedPoint.X - xDim, sharedPoint.Y - yDim, 0);
            return insertPoint;
        }

        private static double getDistance(DBObjectCollection startPartCol, Point3d startPoint)
        {
            double distance = 0;
            if (startPartCol.Count > 0)
            {
                foreach (DBObject objId in startPartCol)
                {
                    if (objId is Line)
                    {
                        Line l = objId as Line;
                        distance = (Math.Pow((startPoint.X - l.EndPoint.X), 2) + Math.Pow((startPoint.Y - l.EndPoint.Y), 2));
                        distance = Math.Sqrt(distance);
                    }
                    else if (objId is Arc)
                    {
                        Arc arcDat = objId as Arc;
                        distance = arcDat.GetDistanceAtParameter(arcDat.GetParameterAtPoint(startPoint));
                    }
                }
            }
            return distance;
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

                    if (sharedPoint == startPoint)
                        return part;
                    else if (sharedPoint == endPoint)
                        return part;
                }
            }
            return null;
        }


        //find slope of line
        //need a catch for vertical lines bc we do not like dividing by 0
        private static double findSlope(Point3d first, Point3d second)
        {
            double slope, x1, x2, y1, y2;
            x1 = Math.Round(first.X, 9);
            x2 = Math.Round(second.X, 9);
            y1 = Math.Round(first.Y, 9);
            y2 = Math.Round(second.Y, 9);

            slope = (y2 - y1) / (x2 - x1);

            return slope;
        }

        //get the y intersect
        private static double findYint(Point3d point, double m)
        {
            double yIntercept = point.Y - m * point.X;
            return yIntercept;
        }
    }
}
