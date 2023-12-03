using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GJKClass
{
    static public (bool, List<Point>) GJK(GameObject first, GameObject second)
    {

        Vector3 direction = (first.transform.position - second.transform.position);
        direction.Normalize();
        direction = roundVector(direction);
        // all vectors should be normalized
        List<Point> simplex = new List<Point>();
        Point firstPoint = new Point();
        getSupport(first, second, direction, ref firstPoint);
        simplex.Add(firstPoint);
        // simplex.Add(getSupport(first, second, direction));

        direction = new Vector3(0, 0, 0) - simplex[0].p;
        direction.Normalize();
        while (true)
        {
            Point newPoint = new Point();
            getSupport(first, second, direction, ref newPoint);
            if (Vector3.Dot(newPoint.p, direction) <= 0)
            {
                return (false, null);
            }
            simplex.Add(newPoint);
            if (ContainsOrigin(ref simplex, ref direction))
            {
                return (true, simplex);
            }
        }

    }

    private static bool SameDirection(Vector3 a, Vector3 b)
    {
        return Vector3.Dot(a, b) > 0;
    }

    private static bool ContainsOrigin(ref List<Point> simplex, ref Vector3 direction)
    {
        int sw = simplex.Count;
        switch (sw)
        {
            case 2: return ContainsOrigin2(ref simplex, ref direction);
            case 3: return ContainsOrigin3(ref simplex, ref direction);
            case 4: return ContainsOrigin4(ref simplex, ref direction);
        }
        return false;
    }

    private static bool ContainsOrigin2(ref List<Point> simplex, ref Vector3 direction)
    {
        Vector3 a = simplex[1].p, b = simplex[0].p;
        Vector3 ab = b - a, ao = -a;
        if (SameDirection(ab, ao))
        {
            direction = Vector3.Cross(Vector3.Cross(ab, ao), ab);
        }
        else
        {
            //never enters here
            simplex.RemoveAt(0);
            direction = ao;
        }
        direction.Normalize();
        return false;
    }

    private static bool ContainsOrigin3(ref List<Point> simplex, ref Vector3 direction)
    {
        Vector3 a = simplex[2].p, b = simplex[1].p, c = simplex[0].p;
        Vector3 ab = b - a, ac = c - a, ao = -a;
        Vector3 abc = Vector3.Cross(ab, ac);
        if (SameDirection(Vector3.Cross(abc, ac), ao))
        {
            simplex.RemoveAt(1);
            return ContainsOrigin2(ref simplex, ref direction);
        }
        else if (SameDirection(Vector3.Cross(ab, abc), ao))
        {
            simplex.RemoveAt(0);
            return ContainsOrigin2(ref simplex, ref direction);
        }
        else
        {
            if (SameDirection(abc, ao))
            {
                direction = abc;
            }
            else
            {
                Point temp = new Point(simplex[0].p, simplex[0].a, simplex[0].b);
                simplex[0] = new Point(b, simplex[1].a, simplex[1].b);
                simplex[1] = temp;
                // simplex[0]= b;
                // simplex[1] = c;
                direction = -abc;
            }
        }

        direction.Normalize();
        return false;
    }

    private static bool ContainsOrigin4(ref List<Point> simplex, ref Vector3 direction)
    {
        Vector3 a = simplex[3].p, b = simplex[2].p, c = simplex[1].p, d = simplex[0].p;
        Vector3 ab = b - a, ac = c - a, ad = d - a, ao = -a;
        Vector3 abc = Vector3.Cross(ab, ac);
        Vector3 acd = Vector3.Cross(ac, ad);
        Vector3 adb = Vector3.Cross(ad, ab);

        if (SameDirection(abc, ao))
        {
            simplex.RemoveAt(0);
            return ContainsOrigin3(ref simplex, ref direction);
        }
        if (SameDirection(acd, ao))
        {
            simplex.RemoveAt(2);
            return ContainsOrigin3(ref simplex, ref direction);
        }
        if (SameDirection(adb, ao))
        {
            simplex.RemoveAt(1);
            return ContainsOrigin3(ref simplex, ref direction);
        }
        return true;
    }


    private static Vector3 getSupport(GameObject first, GameObject second, Vector3 direction, ref Point point)
    {
        Collision collision1 = first.GetComponent<Collision>();
        Collision collision2 = second.GetComponent<Collision>();
        point.a = collision1.findFurthestPoint(direction);
        point.b = collision2.findFurthestPoint(-direction);
        point.p = point.a - point.b;
        //c = roundVector(c);
        return point.p;
    }

    private static float round(float x)
    {
        return Mathf.Round(x * 1000f) / 1000f;
    }

    private static Vector3 roundVector(Vector3 a)
    {
        return new Vector3(round(a.x), round(a.y), round(a.z));
    }
}
