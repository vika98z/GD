using System.Collections.Generic;
using UnityEngine;

public class Memory : MonoBehaviour
{
  public int count;
  public List<float> lastNValues;

  public bool IsStuck;

  public float delta;

  public void Init(int n)
  {
    count = n;
    lastNValues = new List<float>();
    IsStuck = false;
  }

  public void AddValue(float value)
  {
    lastNValues.Add(value);
    if (lastNValues.Count > count)
      lastNValues.RemoveAt(0);
  }

  public void Check()
  {
    if (lastNValues.Count < count)
      return;

    for (var index = 0; index < lastNValues.Count - 1; index++)
    {
      float diff = Mathf.Abs(lastNValues[index] - lastNValues[index + 1]);
      if (diff > delta)
        return;
    }

    IsStuck = true;
  }
}