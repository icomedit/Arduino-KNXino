/** 
 *  MIT License

    Copyright (c) 2021 FABIO Di Michele (https://www.icomed.it)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 **/
 
template<class T>
class EdgeValue
{
  public:
    EdgeValue()
    {
      oldValue = T();
      newValue = T();
    }

    bool isChanged()
    {
      return (newValue != oldValue); 
    }

    bool isTolleranceChanged(float delta)
    {
      float difference = fabs((float)newValue - (float)oldValue);
      return difference >= delta;
    }
    
    void setValue(T value)
    {
      oldValue = newValue;
      newValue = value;
    }

    T getValue()
    {
      return newValue;
    }
    
    private:
        T oldValue;
        T newValue;
};

class Toggle 
{
  public:
    Toggle()
    {
      st = false;
    }

    void setState(bool x)
    {
      st = x;
    }
    
    bool getState()
    {
      return st;
    }

    bool changeStatus()
    {
      st = !st;
      return st;
    }
  private:    
    bool st;
};

class TermostatoRegolabile 
{
  public:
    TermostatoRegolabile()
    {
      sp = 0;
      delta = 0;
      mode = 0;
    }

    void setPoint(float x)
    {
      sp = x;
    }
    
    void setDelta(float d)
    {
      delta = d;
    }

    void setMode(byte b)
    {
      if (b == 0) 
        mode = false;
      else
        mode = true;
    }

    bool getMode()
    {
      return mode;
    }
    
    bool getState(float t)
    {
      float difference = fabs(t - sp);
      if (mode)
        return difference >= delta;
      else
        return difference <= delta;
    }

  private:    
    float sp;
    float delta;
    bool mode;
    
};
