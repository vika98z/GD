using UnityEngine;

public class SingleDegreeJoint : MonoBehaviour
{
  public float MINAngle = -90;
  public float MAXAngle = 90;

  public enum JointDegree
  {
    RotateX = 0,
    RotateY = 1,
    RotateZ = 2
  }

  public JointDegree degreeOfFreedom;
  public Vector3 axis;
  public float StartAngle;
  public Vector3 StartOffset;
  
  private void Start()
  {
    StartAngle = GetValue();
    StartOffset = transform.localPosition;
    
    axis = degreeOfFreedom switch
    {
      JointDegree.RotateX => new Vector3(1, 0, 0),
      JointDegree.RotateY => new Vector3(0, 1, 0),
      JointDegree.RotateZ => new Vector3(0, 0, 1),
      _ => axis
    };
  }

  public void Rotate(float angle)
  {
    if (degreeOfFreedom == JointDegree.RotateY || IsAngleInRange())
      transform.Rotate(axis * angle);

    bool IsAngleInRange()
    {
      float nextAngleValue = CurrentAngle() + angle;
      return nextAngleValue > MINAngle && nextAngleValue < MAXAngle;
    }
  }
  
  public void SimpleRotate(float angle) => 
    transform.Rotate(axis * angle);

  public float CurrentAngle()
  {
    var localEulerAngles = transform.localEulerAngles;
    return degreeOfFreedom switch
    {
      JointDegree.RotateX => WrapAngle(localEulerAngles.x),
      JointDegree.RotateY => WrapAngle(localEulerAngles.y),
      JointDegree.RotateZ => WrapAngle(localEulerAngles.z),
      _ => 0
    };
  }

  private static float WrapAngle(float angle)
  {
    angle %= 360;
    if (angle > 180)
      return angle - 360;

    return angle;
  }
  
  public void SetValue(float value)
  {
    transform.localEulerAngles = degreeOfFreedom switch
    {
      JointDegree.RotateX => new Vector3(value, 0, 0),
      JointDegree.RotateY => new Vector3(0, value, 0),
      JointDegree.RotateZ => new Vector3(0, 0, value),
      _ => transform.localEulerAngles
    };
  }
  
  public float GetValue()
  {
    return transform.transform.localEulerAngles[(int) degreeOfFreedom];
  }
}