#include <iostream>
#include <queue>
#include <vector>

using namespace std;

class Solution {
    public:
        vector<vector<int>> allCellsDistOrder(int rows, int cols, int rCenter, int cCenter) {
            queue<vector<int>> q;
            vector<vector<int>> res;
            bool **visited = new bool*[rows];
            for (int i = 0; i < rows; i++) {
                visited[i] = new bool[cols];
                for (int j = 0; j < cols; j++) {
                    visited[i][j] = false;
                }
            }

            q.push({rCenter,cCenter});  

            while(!q.empty()){
                vector<int> item = q.front();
                q.pop();
                if(!checkBoundary(item,rows,cols)){
                    if (visited[item[0]][item[1]]) continue;
                    res.push_back(item);
                    visited[item[0]][item[1]]=1;
                    q.push({item[0]+1,item[1]});
                    q.push({item[0]-1,item[1]});
                    q.push({item[0],item[1]+1});
                    q.push({item[0],item[1]-1});
                }

                
            }
            return res;
        }

        bool checkBoundary(vector<int> item,int rows, int cols){
            int i;
            int j;
            i = item[0];
            j = item[1];

            if ((i < 0) or (i>=rows) or (j < 0) or (j>=cols)){
                return true;
            }
            return false;
        }
    };

int main(){
    Solution s =  Solution();
    vector<vector<int>>  sol = s.allCellsDistOrder(2,2,0,1);

    for(int i=0;i<sol.size();i++){
        cout<<"["<<sol[i][0]<<","<<sol[i][1]<<"]"<<endl;
    }
    return 0;
}