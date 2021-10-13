using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Kinematics : MonoBehaviour
{
  [SerializeField] private SingleDegreeJoint[] joints;
  [SerializeField] private Transform end;
  [SerializeField] private Transform target;
  [SerializeField] private Memory memory;
  [SerializeField] private CheckCollision[] colliders;
  
  public float learningRate = 5;
  public float distanceThreshold = .5f;

  private float[] _startState;

  public bool IsResetting;

  #region Simplex

  private float a = 1; //коэффициент отражения
  private float b = .5f; //коэффициент сжатия
  private float y = 2; //коэффициент растяжения
  private int n = 5; //количество точек симплекса

  public List<List<float>> simplex = new List<List<float>>
  {
    new List<float>() {60, 0, -60, 80, -80},
    new List<float>() {60, 0, -60, 80, -80},
    new List<float>() {60, 0, -60, 80, -80},
    new List<float>() {60, 0, -60, 80, -80},
    new List<float>() {60, 0, -60, 80, -80},
    new List<float>() {60, 0, -60, 80, -80},
    new List<float>() {60, 0, -60, 80, -80},
  };

  public float[] solution;

  #endregion

  private void Start()
  {
    solution = new float[joints.Length];
    _startState = new float[joints.Length];

    memory.Init(20);

    for (var i = 0; i < joints.Length; i++)
      _startState[i] = joints[i].GetValue();
  }

  private void Update() =>
    InverseKinematics();

  private void InverseKinematics()
  {
    if (IsReachedTheTarget() || IsResetting)
      return;
    
    for (var i = 0; i < joints.Length; i++)
    {
      var joint = joints[i];
      float delta = i == 0 ? 1 : .5f;
      float gradient = Gradient(joint, delta);
      
      if (colliders.All(c => !c.IsBlocked))
        joint.Rotate(-gradient * learningRate);

      if (IsReachedTheTarget())
        return;
    }

    memory.AddValue(Vector3.Distance(end.position, target.transform.position));
    memory.Check();

    if (memory.IsStuck)
      StartCoroutine(Reset());

    bool IsReachedTheTarget() =>
      Vector3.Distance(end.position, target.transform.position) <= distanceThreshold;

    IEnumerator Reset()
    {
      var angleRotateDelta = 1f;
      IsResetting = true;

      //основу и щупальца на конце не крутим, это не обязательно
      for (var i = 1; i < joints.Length - 2; i++)
      {
        var joint = joints[i];

        while (Math.Abs(joint.GetValue() - joint.StartAngle) > angleRotateDelta * 2)
        {
          if (joint.GetValue() > 90) //(joint.GetValue() < 0)
            joint.SimpleRotate(angleRotateDelta);
          else
            joint.SimpleRotate(-angleRotateDelta);

          yield return null;
        }
      }

      memory.Init(20);
      IsResetting = false;
    }
  }

  private float Gradient(SingleDegreeJoint joint, float delta)
  {
    float dist1 = Vector3.Distance(end.position, target.transform.position);
    joint.Rotate(delta);
    float dist2 = Vector3.Distance(end.position, target.transform.position);
    joint.Rotate(-delta);

    return (dist2 - dist1) / delta;
  }

  private Vector3 ForwardKinematics(float[] angles)
  {
    var prevPoint = joints[0].transform.position;
    var rotation = Quaternion.identity;

    for (var i = 1; i < joints.Length; i++)
    {
      rotation *= Quaternion.AngleAxis(angles[i - 1], joints[i - 1].axis);
      var nextPoint = prevPoint + rotation * joints[i].StartOffset;

      prevPoint = nextPoint;
    }

    return prevPoint;
  }

  #region Simplex Methods

  private void MakeSimplexSolution()
  {
    if (!Done())
    {
      for (var i = 0; i < joints.Length; i++)
      {
        var joint = joints[i];
        DownhillSimplexMethod(joint, i);
      }
    }

    for (var i = 0; i < solution.Length; i++)
    {
      joints[i].SetValue(solution[i]);
    }
  }

  private void DownhillSimplexMethod(SingleDegreeJoint joint, int i)
  {
    float xH = 0, xL = 0, xG = 0;

    List<float> values = new List<float>();
    foreach (float angle in simplex[i])
    {
      var value = SimplexCheckFunction(angle, i);
      values.Add(value);
      if (value >= values.Max())
        xH = angle;
      if (value <= values.Min())
        xL = angle;
      if (value <= values.Max() && value >= values.Min())
        xG = angle;
    }

    //Найдём центр тяжести всех точек, за исключением максимальной
    float xC = (xG + xL) / n;

    solution[i] = xC;

    //Отразим точку max относительно massCenter с коэффициентом a
    var xR = (1 + a) * xC - a * xH;

    var fR = SimplexCheckFunction(xR, i);
    var fH = SimplexCheckFunction(xH, i);
    var fL = SimplexCheckFunction(xL, i);
    var fG = SimplexCheckFunction(xG, i);

    var hIndex = simplex[i].FindIndex(x => Math.Abs(x - xH) < 0.01f);

    //направление выбрано удачное и можно попробовать увеличить шаг
    if (fR < fL)
    {
      var xE = (1 - y) * xC + y * xR;
      //fE
      var fE = SimplexCheckFunction(xE, i);

      //можно расширить симплекс до этой точки
      if (fE < fR)
      {
        simplex[i][hIndex] = xE;
        return; //шаг 9
      }
      //переместились слишком далеко

      if (fR < fE)
      {
        simplex[i][hIndex] = xR;
        return; //шаг 9
      }
    }
    //выбор точки неплохой (новая лучше двух прежних)
    else if (fL < fR && fR < fG)
    {
      simplex[i][hIndex] = xR;
      return; //шаг 9
    }
    //меняем местами значения xR и xH. Также нужно поменять местами значения fR и fH
    else if (fG < fR && fR < fH)
    {
      xH = xR;

      //simplex[i][hIndex] = temp;

      fH = fR;
      //шаг 6
    }

    //если fH < fR - просто идём на следующий шаг 6

    //шаг 6 - «Сжатие»
    var xS = b * xH + (1 - b) * xC;
    var fS = SimplexCheckFunction(xS, i);

    if (fS < fH)
    {
      //xH = xS;
      simplex[i][hIndex] = xS;
    }
    //первоначальные точки оказались самыми удачными. Делаем «глобальное сжатие» симплекса — гомотетию к точке с наименьшим значением xL
    else if (fS > fH)
    {
      for (var index = 0; index < simplex[i].Count; index++)
      {
        float x = simplex[i][index];
        if (Math.Abs(x - xL) > 0.01f)
          simplex[i][index] = xL + (x - xL) / 2;
      }
    }
  }

  private bool Done()
  {
    var distance = Vector3.Distance(end.position, target.transform.position);
    return (distance <= distanceThreshold);
  }

  private float SimplexCheckFunction(float value, int i)
  {
    float angle = solution[i];
    solution[i] = value;

    var point = DistanceFromTarget(solution);

    solution[i] = angle;
    return point;
  }

  public float DistanceFromTarget(float[] angles)
  {
    Vector3 point = ForwardKinematics(angles);
    return Vector3.Distance(point, target.position);
  }

  #endregion
}