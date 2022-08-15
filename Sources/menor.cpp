#include "../Headers/menor.h"
//using namespace std;
/*struct poi {
   float poi1, poi2;
};
inline int Comp_poi1(const void* x, const void* b) {
   poi *p1 = (poi *)x, *pnt2 = (poi *)b;
   return (p1->poi1 - pnt2->poi1);
}
inline int Comp_poi2(const void* x, const void* y) {
   poi *pnt1 = (poi *)x, *pnt2 = (poi *)y;
   return (pnt1->poi2 - pnt2->poi2);
}
inline float Distance(poi pnt1, poi pnt2) { // Calculate the distance between two points
   return sqrt( (pnt1.poi1 - pnt2.poi1)*(pnt1.poi1 - pnt2.poi1) +
   (pnt1.poi2 - pnt2.poi2)*(pnt1.poi2 - pnt2.poi2) );
}
float S_Distance(poi P[], int n, poi &pnt1, poi &pnt2) {
   float min = FLT_MAX;
   for (int i = 0; i < n; ++i)
      for (int j = i+1; j < n; ++j)
         if (Distance(P[i], P[j]) < min) {
            min = Distance(P[i], P[j]);
            pnt1.poi1 = P[i].poi1, pnt1.poi2 = P[i].poi2;
            pnt2.poi1 = P[j].poi1, pnt2.poi2 = P[j].poi2;
         }
   return min;
}
inline float Minimum(float poi1, float poi2) {  // Find minimum between two values
   return (poi1 < poi2)? poi1 : poi2;
}
float Closest_dist_Spoint(poi stp[], int s, float dist, poi &pnt1, poi &pnt2) { // Calculate distance beween the closest points
   float Minimum = dist; // Initialize the minimum distance as dist
   qsort(stp, s, sizeof(poi), Comp_poi2);
   for (int i = 0; i < s; ++i)
      for (int j = i+1; j < s && (stp[j].poi2 - stp[i].poi2) < Minimum; ++j)
         if (Distance(stp[i],stp[j]) < Minimum) {
            Minimum = Distance(stp[i], stp[j]);
            pnt1.poi1 = stp[i].poi1, pnt1.poi2 = stp[i].poi2;
            pnt2.poi1 = stp[j].poi1, pnt2.poi2 = stp[j].poi2;
         }
         return Minimum;
}
float Closest_dist(poi P[], poi stp[], int n, poi &pnt1, poi &pnt2) { // Calculate smallest distance.
   static poi pt1, pt2, pt3, pt4;
   if (n <= 3)
      return S_Distance(P, n, pt1, pt2);
   int medium = n/2; // Calculate the mid point
   poi mediumPoint = P[medium];
   float D_Left = Closest_dist(P, stp, medium, pt1, pt2); // D_Left: left of medium point
   float D_Right = Closest_dist(P + medium, stp, n-medium, pt3, pt4); // D_Right: right side of the medium point
   if(D_Left < D_Right) {
      pnt1.poi1 = pt1.poi1; pnt1.poi2 = pt1.poi2; // Store the pair that has smaller distance
      pnt2.poi1 = pt2.poi1; pnt2.poi2 = pt2.poi2;
   } else {
      pnt1.poi1 = pt3.poi1; pnt1.poi2 = pt3.poi2;
      pnt2.poi1 = pt4.poi1; pnt2.poi2 = pt4.poi2;
   }
   float min_dist = Minimum(D_Left, D_Right);
   int j = 0;
   for (int i = 0; i < n; i++)
      if (abs(P[i].poi1 - mediumPoint.poi1) < min_dist)
         stp[j++] = P[i];
      float min_dist_strip = Closest_dist_Spoint(stp, j, min_dist, pt1, pt2);
      float F_Min = min_dist;
      if(min_dist_strip < min_dist) {
         pnt1.poi1 = pt1.poi1; pnt1.poi2 = pt1.poi2;
         pnt2.poi1 = pt2.poi1; pnt2.poi2 = pt2.poi2;
         F_Min = min_dist_strip;
      }
      //for(int i = 0; i < n; ++i)
      std::cout << "Resutado1: " << pnt1.poi1 << ", " << pnt1.poi2 << '\n';
      std::cout << "Resultado2: " << pnt2.poi1 << ", " << pnt2.poi2 << '\n' << "===========================================" << '\n';
   return F_Min;
}
std::vector<Menor::suporte> Menor::calculaMinimo(sf::Vector2f jogador, std::vector<sf::Vector2f> inimigos, int tamanho) {
   poi P[tamanho];
   for(int i = 0; i < tamanho - 1; i++){
      P[i] = {inimigos[i].x, inimigos[i].y};
   }
   P[tamanho - 1] = {jogador.x, jogador.y};
   std::cout << "Jogador: " << P[5].poi1 << ", " << P[5].poi2 << '\n';
   std::cout << "Meta: " << P[0].poi1 << ", " << P[0].poi2 << '\n';
   poi pnt1 = {FLT_MAX, FLT_MAX}, pnt2 = {FLT_MAX, FLT_MAX}; // Closest pair of points in array
   int n = sizeof(P) / sizeof(P[0]);
   qsort(P, n, sizeof(poi), Comp_poi1);
   poi *stp = new poi[n];
   //std::cout << "The closest distance of point in array is: " << Closest_dist(P, stp, n, pnt1, pnt2) << '\n';
   //std::cout << "The closest pair of point in array: (" << pnt1.poi1 << "," << pnt1.poi2 << ") and ("
   //<< pnt2.poi1 << "," << pnt2.poi2 << ")" << '\n';
   float menorDistancia = Closest_dist(P, stp, n, pnt1, pnt2);
   std::vector<Menor::suporte> vetor_resultado;
   struct suporte valores1;
   suporte valores2;
   valores1.x = pnt1.poi1;
   valores1.y = pnt1.poi2;
   valores2.x = pnt2.poi1;
   valores2.y = pnt2.poi2;
   vetor_resultado.push_back(valores1);
   vetor_resultado.push_back(valores2);
   delete[] stp;
   return vetor_resultado;
}

struct poi {
   double poi1, poi2;
};
inline int Comp_poi1(const void* x, const void* b) {
   poi *p1 = (poi *)x, *pnt2 = (poi *)b;
   return (p1->poi1 - pnt2->poi1);
}
inline int Comp_poi2(const void* x, const void* y) {
   poi *pnt1 = (poi *)x, *pnt2 = (poi *)y;
   return (pnt1->poi2 - pnt2->poi2);
}
inline double Distance(poi pnt1, poi pnt2) { // Calculate the distance between two points
   return sqrt( (pnt1.poi1 - pnt2.poi1)*(pnt1.poi1 - pnt2.poi1) +
   (pnt1.poi2 - pnt2.poi2)*(pnt1.poi2 - pnt2.poi2) );
}
double S_Distance(poi P[], int n, poi &pnt1, poi &pnt2) {
   double min = DBL_MAX;
   for (int i = 0; i < n; ++i)
      for (int j = i+1; j < n; ++j)
         if (Distance(P[i], P[j]) < min) {
            min = Distance(P[i], P[j]);
            pnt1.poi1 = P[i].poi1, pnt1.poi2 = P[i].poi2;
            pnt2.poi1 = P[j].poi1, pnt2.poi2 = P[j].poi2;
         }
   return min;
}
inline double Minimum(double poi1, double poi2) {  // Find minimum between two values
   return (poi1 < poi2)? poi1 : poi2;
}
double Closest_dist_Spoint(poi stp[], int s, double dist, poi &pnt1, poi &pnt2) { // Calculate distance beween the closest points
   double Minimum = dist; // Initialize the minimum distance as dist
   qsort(stp, s, sizeof(poi), Comp_poi2);
   for (int i = 0; i < s; ++i)
      for (int j = i+1; j < s && (stp[j].poi2 - stp[i].poi2) < Minimum; ++j)
         if (Distance(stp[i],stp[j]) < Minimum) {
            Minimum = Distance(stp[i], stp[j]);
            pnt1.poi1 = stp[i].poi1, pnt1.poi2 = stp[i].poi2;
            pnt2.poi1 = stp[j].poi1, pnt2.poi2 = stp[j].poi2;
         }
         return Minimum;
}
double Closest_dist(poi P[], poi stp[], int n, poi &pnt1, poi &pnt2) { // Calculate smallest distance.
   static poi pt1, pt2, pt3, pt4;
   if (n <= 3)
      return S_Distance(P, n, pt1, pt2);
   int medium = n/2; // Calculate the mid point
   poi mediumPoint = P[medium];
   double D_Left = Closest_dist(P, stp, medium, pt1, pt2); // D_Left: left of medium point
   double D_Right = Closest_dist(P + medium, stp, n-medium, pt3, pt4); // D_Right: right side of the medium point
   if(D_Left < D_Right) {
      pnt1.poi1 = pt1.poi1; pnt1.poi2 = pt1.poi2; // Store the pair that has smaller distance
      pnt2.poi1 = pt2.poi1; pnt2.poi2 = pt2.poi2;
   } else {
      pnt1.poi1 = pt3.poi1; pnt1.poi2 = pt3.poi2;
      pnt2.poi1 = pt4.poi1; pnt2.poi2 = pt4.poi2;
   }
   double min_dist = Minimum(D_Left, D_Right);
   int j = 0;
   for (int i = 0; i < n; i++)
      if (abs(P[i].poi1 - mediumPoint.poi1) < min_dist)
         stp[j++] = P[i];
      double min_dist_strip = Closest_dist_Spoint(stp, j, min_dist, pt1, pt2);
      double F_Min = min_dist;
      if(min_dist_strip < min_dist) {
         pnt1.poi1 = pt1.poi1; pnt1.poi2 = pt1.poi2;
         pnt2.poi1 = pt2.poi1; pnt2.poi2 = pt2.poi2;
         F_Min = min_dist_strip;
      }
      std::cout << "Resutado1: " << pnt1.poi1 << ", " << pnt1.poi2 << '\n';
      std::cout << "Resultado2: " << pnt2.poi1 << ", " << pnt2.poi2 << '\n' << "===========================================" << '\n';
      return F_Min;
}
std::vector<Menor::suporte> Menor::calculaMinimo(sf::Vector2f jogador, std::vector<sf::Vector2f> inimigos, int tamanho) {
   poi P[tamanho];
   for(int i = 0; i < tamanho - 1; i++){
      P[i] = {(double)(inimigos[i].x), (double)(inimigos[i].y)};
   }
   P[tamanho - 1] = {(double)(jogador.x), (double)(jogador.y)};
   std::cout << "Jogador: " << P[5].poi1 << ", " << P[5].poi2 << '\n';
   std::cout << "Meta: " << P[0].poi1 << ", " << P[0].poi2 << '\n';
   poi pnt1 = {DBL_MAX, DBL_MAX}, pnt2 = {DBL_MAX, DBL_MAX}; // Closest pair of points in array
   int n = sizeof(P) / sizeof(P[0]);
   qsort(P, n, sizeof(poi), Comp_poi1);
   poi *stp = new poi[n];
   //std::cout << "The closest distance of point in array is: " << Closest_dist(P, stp, n, pnt1, pnt2) << '\n';
   //std::cout << "The closest pair of point in array: (" << pnt1.poi1 << "," << pnt1.poi2 << ") and ("
   //<< pnt2.poi1 << "," << pnt2.poi2 << ")" << '\n';
   double menorDistancia = Closest_dist(P, stp, n, pnt1, pnt2);
   std::vector<Menor::suporte> vetor_resultado;
   struct suporte valores1;
   suporte valores2;
   valores1.x = pnt1.poi1;
   valores1.y = pnt1.poi2;
   valores2.x = pnt2.poi1;
   valores2.y = pnt2.poi2;
   vetor_resultado.push_back(valores1);
   vetor_resultado.push_back(valores2);
   delete[] stp;
   return vetor_resultado;
}*/

/*void MergeY(vector<pair<float, float>> &arr, int low, int mid, int high) //Merge function based on y-coordinate
{
    vector<pair<float, float>> temp;
    int i = low;
    int j = mid + 1;

    while (i <= mid && j <= high)
    {
        if (arr[i].second < arr[j].second) //comparing the y-coordinates of the points,the smaller one will come first
        {
            temp.push_back(arr[i++]);
        }
        else if (arr[i].second == arr[j].second && arr[i].first < arr[j].first) //in case the y-coordinates are same then the tie is broken based on the smaller x-coordinate
        {
            temp.push_back(arr[i++]);
        }
        else
        {
            temp.push_back(arr[j++]);
        }
    }

    while (i <= mid) //to push the left over points
    {
        temp.push_back(arr[i++]);
    }
    while (j <= high) //to push the left over points
    {
        temp.push_back(arr[j++]);
    }

    for (int i = low; i <= high; i++) //to write the array again in correct order
    {
        arr[i] = temp[i - low];
    }
}

void MergeSortY(vector<pair<float, float>> &arr, int low, int high) //Mergesort function based on y-coordinate
{
    if (low < high) //if low<high then only enter
    {
        int mid = (low + high) / 2;     //computing the middle value
        MergeSortY(arr, low, mid);      //left recursive call
        MergeSortY(arr, mid + 1, high); //right recursive call
        MergeY(arr, low, mid, high);    //merge procedure at the end
    }
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//Mergesort based on the x-coordinate of the points

void MergeX(vector<pair<float, float>> &arr, int low, int mid, int high) //Merge function based on x-coordinate
{
    vector<pair<float, float>> temp;
    int i = low;
    int j = mid + 1;

    while (i <= mid && j <= high)
    {
        if (arr[i].first < arr[j].first) //comparing the x-coordinates of the points,the smaller one will come first
        {
            temp.push_back(arr[i++]);
        }
        else if (arr[i].first == arr[j].first && arr[i].second < arr[j].second) //in case the x-coordinates are same then the tie is broken based on the smaller y-coordinate
        {
            temp.push_back(arr[i++]);
        }
        else
        {
            temp.push_back(arr[j++]);
        }
    }
    while (i <= mid) //to push the left over points
    {
        temp.push_back(arr[i++]);
    }
    while (j <= high)
    {
        temp.push_back(arr[j++]); //to push the left over elements
    }

    for (int i = low; i <= high; i++) //to write the array again in correct order
    {
        arr[i] = temp[i - low];
    }
}

void MergeSortX(vector<pair<float, float>> &arr, int low, int high)
{
    if (low < high) //if low<high then only enter
    {
        int mid = (low + high) / 2;     //computing the middle value
        MergeSortX(arr, low, mid);      //left recursive call
        MergeSortX(arr, mid + 1, high); //right recursive call
        MergeX(arr, low, mid, high);    //merge procedure at the end
    }
}

//the function find_distance() computes the distance between two pairs of points

float find_distance(pair<float, float> p1, pair<float, float> p2, pair<float, float> &min1, pair<float, float> &min2)
{
    //if new pair has shorter distance compared to the old pair of points, then update the closest pairs
    if (sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second)) < sqrt((min1.first - min2.first) * (min1.first - min2.first) + (min1.second - min2.second) * (min1.second - min2.second)))
    {
        min1 = p1;
        min2 = p2;
    }
    //returns the distance computed between the two points
    return sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
}

//main function that computes the distance between the closest pair of points

float find_smallest_distance(vector<pair<float, float>> sorted_x, vector<pair<float, float>> sorted_y, pair<float, float> &min1, pair<float, float> &min2)
{
   cout << "Entrou aqui\n"; 
    int n = sorted_x.size(); //n contains the number of points

    if (n <= 1) //if n<=1 then no need to further divide
    {
        return FLT_MAX;
    }

    pair<float, float> p1, p2, p3, p4 = {FLT_MAX, FLT_MAX}; //initializing the dummy pairs as FLT_MAX
    vector<pair<float, float>> left_x;                      //left_x has the value when x-coordinate <=x_mid
    vector<pair<float, float>> right_x;                     //right_x has the value when x-coordinate > x_mid

    for (int i = 0; i < n; i++) //put the points in left_x and right_x
    {
        if (i < n / 2)
        {
            left_x.push_back(sorted_x[i]);
        }
        else
        {
            right_x.push_back(sorted_y[i]);
        }
    }
    int middle = left_x.back().first;             //compute the median value based on the x-coordinate
    vector<pair<float, float>> left_y, right_y; //Points in left_y have the value when y-coordinate <=x_mid and right_y contains the rest of the coordinates
    for (auto x : sorted_y) //fill the points in left_y and right_y
    {
        if (x.first <= middle)
        {
            left_y.push_back(x);
        }
        else
        {
            right_y.push_back(x);
        }
    }
  
    float d1 = find_smallest_distance(left_x, left_y, p1, p2);   //recursive call on the left region
    float d2 = find_smallest_distance(right_x, right_y, p3, p4); //recursive call on the right region
   cout << "Saiu aqui\n";
    if (d1 < d2) //update the closest points based on the minimum distance between them
    {
        min1 = p1;
        min2 = p2;
    }
    else
    {
        min1 = p3;
        min2 = p4;
    }

    float d = min(d1, d2); //d contains the minimum distance from both the regions

    vector<pair<float, float>> band; //band will contain all the elements that are distance d apart from the mid_x value in both direction

    for (auto x : sorted_y) //populating the band vector
    {
        if (x.first > middle - d)
            band.push_back(x);
    }
    for (int i = 0; i < band.size(); i++) //O(n) time loop
    {
        for (int j = i + 1; j < band.size() && band[j].second < band[i].second + d; j++) //this will take O(1) guaranteed, for each point it will check for it's neighbouring 7 points at max
        {
            if (find_distance(band[i], band[j], p1, p2) < d) //if found some better distance and points then simply update
            {
                min1 = band[i];
                min2 = band[j];
                d = find_distance(band[i], band[j], p1, p2);
            }
        }
    }
   
    return d; //return the closest distance found
}


vector<Menor::suporte> Menor::calculaMinimo(sf::Vector2f jogador, vector<sf::Vector2f> inimigos, int tamanho){
   vector<pair<float, float>> vec;
   for(int i = 0; i < tamanho - 1; ++i){
      vec.push_back({(inimigos[i].x), (inimigos[i].y)});
   }
   vec.push_back({(jogador.x), (jogador.y)});
   vector<pair<float, float>> vetorSuporteX = vec;
   vector<pair<float, float>> vetorSuporteY = vec;
   MergeSortX(vetorSuporteX, 0, vetorSuporteX.size() - 1);          //Applying the merge sort on the x-coordinate of the points
    vector<pair<float, float>> sorted_x = vetorSuporteX; //sorted_x contains the points sorted on the basics of the x-coordinates
   
    MergeSortY(vetorSuporteY, 0, vetorSuporteY.size() - 1);          //Applying the merge sort on the y-coordinate of the points
    vector<pair<float, float>> sorted_y = vetorSuporteY; //sorted_y contains the points sorted on the basics of the y-coordinates
      
    pair<float, float> min1 = {FLT_MAX, FLT_MAX}; //min1 and min2 contains the closest points found at the end
    pair<float, float> min2 = {FLT_MAX, FLT_MAX};

    float x = find_smallest_distance(sorted_x, sorted_y, min1, min2); //find_smallest_distance() function will return the smallest distance between two points
   //cout << "entrou\n";
   // cout << "The closest pair of points are (" << min1.first << "," << min1.second << ") "
   //      << "and "
   //      << "(" << min2.first << "," << min2.second << ") " << endl;
   //cout << vec.size() << '\n';
    //cout << "The distance between them is " << x << " units" << endl;
   vector<Menor::suporte> vetor_resultado;
   struct suporte valores1;
   suporte valores2;
   valores1.x = min1.first;
   valores1.y = min1.second;
   valores2.x = min2.first;
   valores2.y = min2.second;
   vetor_resultado.push_back(valores1);
   vetor_resultado.push_back(valores2);
   //cout << "saiu\n";
   return vetor_resultado;
}*/

struct point
{
   // data members
   double x, y;    // point coordinates

   // constructor
   point(double theX = 0, double theY = 0)
   {
      x = theX;
      y = theY;
   }
};

struct point1 : point
{// Point with id, comparaisons done using x-coordinates.
   // data member
   int id;         // point identifier
  
   // constructor
   point1(double theX = 0, double theY = 0, int theID = 0)
   {
      x = theX;
      y = theY;
      id = theID;
   }
   //重写了这个方法，则mergeSort在调用point的operator<的时候，会先隐式转型成double然后进行比较
   //所以，其实重写operator<也可以
   operator double() const {return x;} 
};

struct point2 : point
{// Point with an integer field, comparisons done using y-coordinates.
   int p;          // index to same point in array X

   // constructor
   point2(double theX = 0, double theY = 0, int theP = 0)
   {
      x = theX;
      y = theY;
      p = theP;
   }

   operator double() {return y;}
};

struct pointPair
{// Pairs of points and their distance.
   // data members
   point1 a;       // one of the points
   point1 b;       // the other point
   double dist;    // distance between a and b

   // constructor
   pointPair(point1& theA, point1& theB, double theDist)
   {
      a = theA;
      b = theB;
      dist = theDist;
   }
};

double dist(const point& u, const point& v)
{// return distance between points u and v.
   double dx = u.x - v.x;
   double dy = u.y - v.y;
   return sqrt(dx * dx + dy * dy);
}

pointPair closestPair(point1 x[], point2 y[], point2 z[], int l, int r)
{// x[l:r] points sorted by x-coordinate, r > l.
 // y[l:r] points sorted by y-coordinate.
 // z[l:r] is used for work space.
 // Return closest pair of points in x[l:r].
   if (r - l == 1)  // only two points
      return pointPair(x[l], x[r], dist(x[l], x[r]));

   if (r - l == 2)
   {// three points
      // compute distance between all pairs
      double d1 = dist(x[l], x[l + 1]);
      double d2 = dist(x[l + 1], x[r]);
      double d3 = dist(x[l], x[r]);
      // find closest pair
      if (d1 <= d2 && d1 <= d3)
         return pointPair(x[l], x[l + 1], d1);
      if (d2 <= d3)
         return pointPair(x[l + 1], x[r], d2);
      else
         return pointPair(x[l], x[r], d3);
   }

   // more than three points, divide into two
   int m = (l + r) / 2;    // x[l:m] in A, rest in B

   // create sorted-by-y lists in z[l:m] & z[m+1:r]
   int f = l,      // cursor for z[l:m]
       g = m + 1;  // cursor for z[m+1:r]
   for (int i = l; i <= r; i++)
      if (y[i].p > m) z[g++] = y[i];
      else z[f++] = y[i];

   // solve the two parts
   pointPair best = closestPair(x, z, y, l, m);
   pointPair right = closestPair(x, z, y, m + 1, r);

   // make best closer pair of the two
   if (right.dist < best.dist)
      best = right;

   merge(z, y, l, m, r);   // reconstruct y

   // put points within best.d of midpoint in z
   int k = l;                        // cursor for z
   for (int i = l; i <= r; i++)
      if (fabs(x[m].x - y[i].x) < best.dist)
         z[k++] = y[i];

   // search for closer category 3 pair
   // by checking all pairs from z[l:k-1]
   for (int i = l; i < k; i++)
   {
      for (int j = i + 1; j < k && z[j].y - z[i].y < best.dist; j++)
      {
         double dp = dist(z[i], z[j]);
         if (dp < best.dist) // closer pair found
            best = pointPair(x[z[i].p], x[z[j].p], dp);
      }
   }
   return best;
}

pointPair closestPair(point1 x[], int numberOfPoints)
{// Return closest pair of points in x[0:numberOfPoints-1].
 // Throw an exception if fewer than two points.
   int n = numberOfPoints;
   if (n < 2)
      std::cout << "Number of points must be > 1\n";

   // sort on x-coordinate
   mergeSort(x, n);

   // create a point array sorted on y-coordinate
   point2 *y = new point2 [n];
   for (int i = 0; i < n; i++)
      // copy point i from x to y and index it
      y[i] = point2(x[i].x, x[i].y, i);
   mergeSort(y, n);  // sort on y-coordinate

   // create a temporary array
   point2 *z = new point2 [n];

   // find closest pair
   return closestPair(x, y, z, 0, n - 1);
}
  
std::vector<Menor::suporte> Menor::calculaMinimo(sf::Vector2f jogador, std::vector<sf::Vector2f> inimigos, int tamanho){
   point1 *x = new point1 [tamanho];
   for(int i = 0; i < tamanho - 1; i++){
      x[i] = point1((double)(inimigos[i].x), (double)(inimigos[i].y), i+1);
   }
   x[tamanho - 1] = point1((double)(jogador.x), (double)(jogador.y), tamanho);
   pointPair best = closestPair(x, tamanho);
   vector<Menor::suporte> vetor_resultado;
   struct suporte valores1;
   suporte valores2;
   valores1.x = best.a.x;
   valores1.y = best.a.y;
   valores2.x = best.b.x;
   valores2.y = best.b.y;
   vetor_resultado.push_back(valores1);
   vetor_resultado.push_back(valores2);
   //cout << "saiu\n";
   return vetor_resultado;
}
/*void main(void)
{
   cout << "Enter number of points" << endl;
   int n;
   cin >> n;
   point1 *x = new point1 [n];

   for (int i = 0; i < n; i++)
   {
      cout << "Enter point " << i + 1 << endl;
      double xcoord, ycoord;
      cin >> xcoord >> ycoord;
      x[i] = point1(xcoord, ycoord, i + 1);
   }

   pointPair best = closestPair(x, n);
   cout << "Closest points are " << best.a.id <<
                      " and " << best.b.id << endl;
   cout << "Their distance is " << best.dist << endl;
   vector<Menor::suporte> vetor_resultado;
   
}*/
