/**
 * Inter IIT Tech Meet - Candidate Solver
 * Single-file autonomous client: robust TCP + global planning + pure-pursuit control.
 * Handles all 4 scenarios. STL + POSIX sockets only.
 */
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <queue>
#include <limits>
#include <algorithm>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---------------- utils ----------------
static inline double clampD(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline double normAng(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
static inline double angDiff(double a, double b) { return std::abs(normAng(a - b)); }

struct VehicleParams {
    double length = 4.0, width = 1.8, wheelbase = 2.5;
    double rear_overhang = 0.8, front_overhang = 0.7;
    double max_steer = 0.60, max_speed = 2.5, min_speed = -1.5;
    double max_steer_rate = 1.0;
    double minRadius() const { return wheelbase / std::tan(max_steer); }
};
struct Pose2D { double x=0,y=0,yaw=0,v=0,delta=0; int dir=+1; };

// ---------------- grid map ----------------
struct GridMap {
    int cols=0, rows=0;
    double res=0.2, ox=-50.0, oy=-50.0;
    std::vector<uint8_t> occ;   // 0 free 1 occ
    std::vector<float> dist;    // meters to nearest obstacle (chamfer)
    bool worldToGrid(double wx,double wy,int& gx,int& gy) const {
        gx = (int)std::floor((wx-ox)/res); gy=(int)std::floor((wy-oy)/res);
        return gx>=0&&gx<cols&&gy>=0&&gy<rows;
    }
    void gridToWorld(int gx,int gy,double& wx,double& wy) const {
        wx=ox+(gx+0.5)*res; wy=oy+(gy+0.5)*res;
    }
    bool occCell(int gx,int gy) const {
        if(gx<0||gx>=cols||gy<0||gy>=rows) return true;
        return occ[gy*cols+gx]!=0;
    }
    bool occWorld(double wx,double wy) const { int gx,gy; if(!worldToGrid(wx,wy,gx,gy)) return true; return occCell(gx,gy); }
    float distWorld(double wx,double wy) const { int gx,gy; if(!worldToGrid(wx,wy,gx,gy)) return 0; return dist[gy*cols+gx]; }
    void buildDist() {
        const float INF=1e9f;
        dist.assign(cols*rows, INF);
        for(int i=0;i<cols*rows;i++) if(occ[i]) dist[i]=0;
        const float SQ=(float)std::sqrt(2.0);
        // forward
        for(int r=0;r<rows;r++) for(int c=0;c<cols;c++){
            int i=r*cols+c; if(dist[i]==0) continue;
            float best=dist[i];
            if(c>0) best=std::min(best,dist[i-1]+1);
            if(r>0) best=std::min(best,dist[i-cols]+1);
            if(c>0&&r>0) best=std::min(best,dist[i-cols-1]+SQ);
            if(c+1<cols&&r>0) best=std::min(best,dist[i-cols+1]+SQ);
            dist[i]=best;
        }
        for(int r=rows-1;r>=0;r--) for(int c=cols-1;c>=0;c--){
            int i=r*cols+c; float best=dist[i];
            if(c+1<cols) best=std::min(best,dist[i+1]+1);
            if(r+1<rows) best=std::min(best,dist[i+cols]+1);
            if(c+1<cols&&r+1<rows) best=std::min(best,dist[i+cols+1]+SQ);
            if(c>0&&r+1<rows) best=std::min(best,dist[i+cols-1]+SQ);
            dist[i]=best;
        }
        for(auto& d:dist) d*=(float)res;
    }
};

// exact replica of simulator bicycle step
static Pose2D stepModel(const Pose2D& s,double tv,double td,double dt,const VehicleParams& p){
    Pose2D n=s;
    tv=clampD(tv,p.min_speed,p.max_speed); td=clampD(td,-p.max_steer,p.max_steer);
    double derr=td-s.delta, mx=p.max_steer_rate*dt;
    n.delta=s.delta+clampD(derr,-mx,mx);
    n.v=tv;
    n.x+=n.v*std::cos(s.yaw)*dt; n.y+=n.v*std::sin(s.yaw)*dt;
    n.yaw=normAng(n.yaw+(n.v/p.wheelbase)*std::tan(n.delta)*dt);
    return n;
}

// footprint collision via body sample points vs occ + clearance margin
static bool collideAt(double x,double y,double yaw,const GridMap& m,const VehicleParams& p,double clearance){
    double xr=-p.rear_overhang, xf=p.wheelbase+p.front_overhang, hw=p.width/2.0;
    double cy=std::cos(yaw), sy=std::sin(yaw);
    // sample lattice over body
    for(double lx=xr; lx<=xf+1e-6; lx+=0.4){
        for(double ly=-hw; ly<=hw+1e-6; ly+=0.45){
            double wx=x+lx*cy-ly*sy, wy=y+lx*sy+ly*cy;
            int gx,gy;
            if(!m.worldToGrid(wx,wy,gx,gy)) return true;
            if(m.occCell(gx,gy)) return true;
            if(m.dist[gy*m.cols+gx] < clearance) return true;
        }
    }
    // corners explicitly (finer)
    double cxs[4]={xf,xf,xr,xr}, cys[4]={hw,-hw,-hw,hw};
    for(int i=0;i<4;i++){
        double wx=x+cxs[i]*cy-cys[i]*sy, wy=y+cxs[i]*sy+cys[i]*cy;
        if(m.occWorld(wx,wy)) return true;
    }
    return false;
}
static bool segCollision(double x0,double y0,double y0w,double x1,double y1,double y1w,
                         const GridMap& m,const VehicleParams& p,double clr){
    double d=std::hypot(x1-x0,y1-y0);
    int n=(int)std::ceil(d/0.15)+1; n=std::max(n,2); n=std::min(n,60);
    for(int i=0;i<=n;i++){ double t=(double)i/n;
        double x=x0+(x1-x0)*t, y=y0+(y1-y0)*t;
        double a=y0w+normAng(y1w-y0w)*t;
        if(collideAt(x,y,a,m,p,clr)) return true;
    }
    return false;
}

// ---------------- Dijkstra from goal (inflated) ----------------
static std::vector<float> dijkstraFromGoal(const GridMap& m,int ggx,int ggy,double inflation){
    const float INF=1e9f;
    std::vector<float> cost(m.cols*m.rows, INF);
    std::vector<char> blocked(m.cols*m.rows,0);
    for(int i=0;i<m.cols*m.rows;i++) if(m.dist[i]<inflation) blocked[i]=1;
    if(ggx<0||ggx>=m.cols||ggy<0||ggy>=m.rows) return cost;
    // allow goal cell even if marginally inflated (parking slots are tight)
    blocked[ggy*m.cols+ggx]=0;
    using QI=std::pair<float,int>;
    std::priority_queue<QI,std::vector<QI>,std::greater<QI>> pq;
    int gi=ggy*m.cols+ggx; cost[gi]=0; pq.emplace(0,gi);
    const int dx[8]={1,1,0,-1,-1,-1,0,1}, dy[8]={0,1,1,1,0,-1,-1,-1};
    const float w[8]={1,(float)std::sqrt(2),1,(float)std::sqrt(2),1,(float)std::sqrt(2),1,(float)std::sqrt(2)};
    while(!pq.empty()){
        auto [c,i]=pq.top(); pq.pop();
        if(c>cost[i]) continue;
        int cx=i%m.cols, cy=i/m.cols;
        for(int k=0;k<8;k++){ int nx=cx+dx[k],ny=cy+dy[k];
            if(nx<0||nx>=m.cols||ny<0||ny>=m.rows) continue;
            int ni=ny*m.cols+nx; if(blocked[ni]) continue;
            // prevent corner cutting
            if(dx[k]!=0&&dy[k]!=0){ if(blocked[cy*m.cols+nx]||blocked[ny*m.cols+cx]) continue; }
            // clearance-weighted cost: prefer corridor centers (critical for long car body)
            float dcc=m.dist[ni];
            float extra=(dcc<2.0f)?2.0f*(2.0f-dcc):0.0f;
            float nc=c+w[k]*(float)m.res*(1.0f+extra);
            if(nc<cost[ni]){ cost[ni]=nc; pq.emplace(nc,ni); }
        }
    }
    return cost;
}
// gradient-descent path extraction on dijkstra field
static std::vector<Pose2D> descentPath(const GridMap& m,const std::vector<float>& cost,
                                       double sx,double sy,double gx,double gy){
    std::vector<Pose2D> path;
    int cgx,cgy; m.worldToGrid(sx,sy,cgx,cgy);
    int ggx,ggy; m.worldToGrid(gx,gy,ggx,ggy);
    const float INF=1e9f;
    int cx=cgx, cy=cgy, guard=0;
    double wx,wy; m.gridToWorld(cx,cy,wx,wy);
    path.push_back({wx,wy,0,0,0,1});
    const int dx[8]={1,1,0,-1,-1,-1,0,1}, dy[8]={0,1,1,1,0,-1,-1,-1};
    while(guard++<20000){
        if(std::hypot(cx-ggx,cy-ggy)<=1) break;
        int ci=cy*m.cols+cx;
        float best=cost[ci]; int bx=cx,by=cy;
        for(int k=0;k<8;k++){ int nx=cx+dx[k],ny=cy+dy[k];
            if(nx<0||nx>=m.cols||ny<0||ny>=m.rows) continue;
            float c=cost[ny*m.cols+nx];
            if(c<best){best=c;bx=nx;by=ny;}
        }
        if(bx==cx&&by==cy) break; // stuck
        cx=bx;cy=by;
        m.gridToWorld(cx,cy,wx,wy);
        // sparse: push every cell (will downsample later)
        path.push_back({wx,wy,0,0,0,1});
        if(cost[cy*m.cols+cx]>=INF/2) break;
    }
    // ensure goal appended
    path.push_back({gx,gy,0,0,0,1});
    return path;
}

// append exact goal then smooth yaw over last points (avoid infeasible kink)
static void appendGoalSmooth(std::vector<Pose2D>& out,const Pose2D& goal){
    out.push_back({goal.x,goal.y,goal.yaw,0,0,out.empty()?1:out.back().dir});
    if(out.size()>=7){
        double base=out[out.size()-6].yaw;
        double d=normAng(goal.yaw-base);
        for(size_t k=out.size()-5;k<out.size();k++){
            double t=(double)(k-(out.size()-5))/4.0; // k=size-1 -> t=1 exact goal yaw
            out[k].yaw=normAng(base+d*t*t);
        }
    }
}
// ---------------- Hybrid A* (parking) ----------------
struct HANode{ double x,y,yaw,g; int parent; int8_t si; int8_t dir; };
static std::vector<Pose2D> hybridAStar(const Pose2D& start,const Pose2D& goal,
    const GridMap& m,const VehicleParams& p,const std::vector<float>& hmap,
    bool allowReverse,double clearance,int maxExp=250000){
    const double steers[5]={-1.0,-0.5,0.0,0.5,1.0}; // scaled by max_steer
    const double PRIM=1.1, SUBN=5, SPEED=1.1;
    const int NT=36;
    double cellA=0.6;
    int nx=(int)std::ceil(100.0/cellA)+2, ny=nx;
    auto vidx=[&](double x,double y,double yaw){ int ix=(int)((x+50.0)/cellA), iy=(int)((y+50.0)/cellA);
        int it=((int)std::round((yaw+M_PI)/(2*M_PI)*NT))%NT; if(it<0)it+=NT;
        ix=std::max(0,std::min(nx-1,ix)); iy=std::max(0,std::min(ny-1,iy));
        return (iy*nx+ix)*NT+it; };
    std::vector<float> best((size_t)nx*ny*NT, 1e9f);
    std::vector<HANode> nodes; nodes.reserve(50000);
    using QI=std::pair<double,int>;
    std::priority_queue<QI,std::vector<QI>,std::greater<QI>> open;
    auto hfn=[&](double x,double y,double yaw)->double{
        int gx,gy; double hE=std::hypot(x-goal.x,y-goal.y);
        double h=hE;
        if(m.worldToGrid(x,y,gx,gy)){ float d=hmap[gy*m.cols+gx]; if(d<1e8) h=std::max(hE*0.5,(double)d); }
        h += angDiff(yaw,goal.yaw)*0.6;
        return h;
    };
    nodes.push_back({start.x,start.y,normAng(start.yaw),0,-1,2,1});
    open.emplace(hfn(start.x,start.y,start.yaw),0);
    best[vidx(start.x,start.y,start.yaw)]=0;
    if(collideAt(start.x,start.y,start.yaw,m,p,0.0)) std::cerr<<"[Plan] start in collision!\n";
    int exp=0, goalIdx=-1;
    std::vector<int> dirs={1}; if(allowReverse) dirs={1,-1};
    while(!open.empty()&&exp<maxExp){
        auto [f,ii]=open.top(); open.pop();
        // lazy skip if stale? (compare g via f-h approx) - skip check, allow re-expansion but cap
        const HANode cur=nodes[ii];
        exp++;
        double dg=std::hypot(cur.x-goal.x,cur.y-goal.y);
        if(dg<0.4&&angDiff(cur.yaw,goal.yaw)<0.2){
            // accept only if goal lies ahead in travel direction (no overshoot+jump-back)
            double dot=(goal.x-cur.x)*std::cos(cur.yaw)+(goal.y-cur.y)*std::sin(cur.yaw);
            if(dot*(double)cur.dir>-0.05){ goalIdx=ii; break; }
        }
        // analytic connect when close (greedy pursuit, per-step direction)
        if(dg<8.0&&exp%800==0){
            // greedy pursuit simulation toward goal
            Pose2D s{cur.x,cur.y,cur.yaw,0,cur.dir>0?0:0}; s.delta=0;
            std::vector<std::pair<Pose2D,int>> trial; trial.emplace_back(s,1);
            bool ok=true;
            Pose2D q=s;
            for(int k=0;k<80;k++){
                double dpos=std::hypot(goal.x-q.x,goal.y-q.y);
                double want=std::atan2(goal.y-q.y,goal.x-q.x);
                double e=normAng(want-q.yaw);
                // choose direction: prefer current dir sign; flip if goal behind and reverse allowed
                double dirS = 1.0;
                if(allowReverse && std::abs(e)>M_PI/2){ dirS=-1.0; e=normAng(e+M_PI); }
                double w=clampD(1.0-dpos/4.0,0.0,1.0); // near goal: also align yaw
                double ey=normAng(goal.yaw-q.yaw)*(dirS>0?1.0:-1.0);
                double td=clampD(1.8*e+1.4*ey*w,-p.max_steer,p.max_steer);
                q=stepModel(q,dirS*SPEED,td,0.25/SPEED*1.0,p);
                if(collideAt(q.x,q.y,q.yaw,m,p,clearance)){ ok=false; break; }
                trial.emplace_back(q,(int)dirS);
                if(std::hypot(q.x-goal.x,q.y-goal.y)<0.3&&angDiff(q.yaw,goal.yaw)<0.2){
                    // stitch: build full path from ii + trial
                    std::vector<HANode> chain; int t=ii; while(t>=0){chain.push_back(nodes[t]); t=nodes[t].parent;}
                    std::reverse(chain.begin(),chain.end());
                    std::vector<Pose2D> out;
                    for(auto& n:chain) out.push_back({n.x,n.y,n.yaw,0,0,n.dir});
                    for(size_t k2=1;k2<trial.size();k2++){ auto qq=trial[k2].first; qq.dir=trial[k2].second; out.push_back({qq.x,qq.y,qq.yaw,0,0,qq.dir}); }
                    appendGoalSmooth(out,goal);
                    std::cerr<<"[Plan] analytic connect ok at exp "<<exp<<"\n";
                    return out;
                }
            }
            (void)ok;
        }
        for(int d:dirs){
            for(int s2=0;s2<5;s2++){
                double td=steers[s2]*p.max_steer;
                Pose2D q{cur.x,cur.y,cur.yaw,0,0}; q.delta=0; // start delta 0 approx (ignore history for speed)
                // NOTE: using delta=0 start slightly optimistic re steer-rate; acceptable + checked by controller margin
                bool hit=false;
                double gadd=0;
                for(int k=0;k<(int)SUBN;k++){
                    q=stepModel(q,d*SPEED,td,(PRIM/SUBN)/SPEED,p);
                    if(collideAt(q.x,q.y,q.yaw,m,p,clearance)){hit=true;break;}
                    gadd+=(PRIM/SUBN)*(d<0?2.2:1.0);
                }
                if(hit) continue;
                gadd += std::abs(steers[s2])*0.15 + (s2!=nodes[ii].si?0.1:0);
                if(d!=nodes[ii].dir) gadd+=3.0; // cusp (direction switch) penalty -> fewer, cleaner maneuvers
                // clearance penalty (push path away from obstacles)
                float dc=m.distWorld(q.x,q.y);
                if(dc<0.9) gadd += (0.9-dc)*4.0;
                double ng=cur.g+gadd;
                int vi=vidx(q.x,q.y,q.yaw);
                if(ng<best[vi]-1e-6){
                    best[vi]=(float)ng;
                    double h=hfn(q.x,q.y,q.yaw);
                    nodes.push_back({q.x,q.y,q.yaw,ng,ii,(int8_t)s2,(int8_t)d});
                    open.emplace(ng+h,(int)nodes.size()-1);
                }
            }
        }
    }
    if(goalIdx<0){ std::cerr<<"[Plan] hybrid failed exp="<<exp<<"\n"; return {}; }
    std::cerr<<"[Plan] hybrid ok exp="<<exp<<" nodes="<<nodes.size()<<"\n";
    // reconstruct (endpoints) then densify edges by re-simulation
    std::vector<int> chain; for(int t=goalIdx;t>=0;t=nodes[t].parent) chain.push_back(t);
    std::reverse(chain.begin(),chain.end());
    std::vector<Pose2D> out;
    out.push_back({nodes[chain[0]].x,nodes[chain[0]].y,nodes[chain[0]].yaw,0,0,nodes[chain[0]].dir});
    for(size_t i=1;i<chain.size();i++){
        const HANode& pr=nodes[chain[i-1]]; const HANode& cu=nodes[chain[i]];
        double td=((double)(int)cu.si-2)*0.5*p.max_steer; // inverse of steers table: si 0..4 -> -1,-0.5,0,0.5,1
        // NOTE si mapping: steers[si]; recompute exactly:
        td=steers[(int)cu.si]*p.max_steer;
        Pose2D q{pr.x,pr.y,pr.yaw,0,0}; q.delta=0;
        for(int k=0;k<(int)SUBN;k++){ q=stepModel(q,(double)cu.dir*SPEED,td,(PRIM/SUBN)/SPEED,p);
            q.dir=cu.dir; out.push_back({q.x,q.y,q.yaw,0,0,cu.dir}); }
    }
    appendGoalSmooth(out,goal);
    return out;
}

// path helpers
static std::vector<Pose2D> downsample(const std::vector<Pose2D>& in,double step){
    if(in.empty()) return in;
    std::vector<Pose2D> o; o.push_back(in[0]);
    double acc=0;
    for(size_t i=1;i<in.size();i++){ acc+=std::hypot(in[i].x-in[i-1].x,in[i].y-in[i-1].y);
        if(acc>=step){ o.push_back(in[i]); acc=0; } }
    if(o.back().x!=in.back().x||o.back().y!=in.back().y) o.push_back(in.back());
    return o;
}
static std::vector<Pose2D> smoothTangent(std::vector<Pose2D> p){
    for(size_t i=1;i+1<p.size();i++){
        p[i].yaw=std::atan2(p[i+1].y-p[i-1].y,p[i+1].x-p[i-1].x);
    }
    if(p.size()>=2) p[0].yaw=p[1].yaw, p.back().yaw=p[p.size()-2].yaw;
    return p;
}
static void assignSpeeds(std::vector<Pose2D>& path,const VehicleParams& pr,double cruise){
    for(size_t i=0;i<path.size();i++){
        double v=cruise*path[i].dir;
        // slow near cusps
        if(i>0&&path[i].dir!=path[i-1].dir) v=0;
        if(i+1<path.size()&&path[i].dir!=path[i+1].dir) v=0;
        // slow near end
        double dEnd=0; for(size_t j=i;j+1<path.size();j++) dEnd+=std::hypot(path[j+1].x-path[j].x,path[j+1].y-path[j].y);
        if(dEnd<3.0) v=clampD(v,-0.7,0.7);
        if(dEnd<1.0) v=clampD(v,-0.35,0.35);
        path[i].v=v;
        // curvature -> delta
        if(i>0&&i+1<path.size()){
            double dy=normAng(path[i+1].yaw-path[i-1].yaw);
            double ds=std::hypot(path[i+1].x-path[i-1].x,path[i+1].y-path[i-1].y)+1e-6;
            double kap=dy/ds;
            path[i].delta=clampD(std::atan(pr.wheelbase*kap),-pr.max_steer,pr.max_steer);
            if(path[i].dir<0) path[i].delta=-path[i].delta*0.9; // approx for reverse (controller overrides anyway)
        }
    }
    if(!path.empty()) path.back().v=0;
}

// min distance-to-obstacle over vehicle body samples (for speed control)
static float bodyMinDist(double x,double y,double yaw,const GridMap& m,const VehicleParams& p){
    double xr=-p.rear_overhang, xf=p.wheelbase+p.front_overhang, hw=p.width/2.0;
    double cy=std::cos(yaw), sy=std::sin(yaw);
    float best=1e9f;
    for(double lx=xr; lx<=xf+1e-6; lx+=0.8){
        for(double ly=-hw; ly<=hw+1e-6; ly+=0.9){
            double wx=x+lx*cy-ly*sy, wy=y+lx*sy+ly*cy;
            int gx,gy; if(!m.worldToGrid(wx,wy,gx,gy)) return 0;
            best=std::min(best,m.dist[gy*m.cols+gx]);
        }
    }
    return best;
}
// push footprint-colliding waypoints away from obstacles along dist gradient
static void repairFootprint(std::vector<Pose2D>& path,const GridMap& m,const VehicleParams& p){
    for(int it=0;it<25;it++){
        bool any=false;
        for(size_t i=1;i+1<path.size();i++){
            if(!collideAt(path[i].x,path[i].y,path[i].yaw,m,p,0.12)){
                // verify edge midpoints too via segment checks to neighbors
                if(!segCollision(path[i-1].x,path[i-1].y,path[i-1].yaw,path[i].x,path[i].y,path[i].yaw,m,p,0.10)) continue;
            }
            any=true;
            double e=0.3;
            double gx=(m.distWorld(path[i].x+e,path[i].y)-m.distWorld(path[i].x-e,path[i].y))/(2*e);
            double gy=(m.distWorld(path[i].x,path[i].y+e)-m.distWorld(path[i].x,path[i].y-e))/(2*e);
            double n=std::hypot(gx,gy)+1e-9;
            path[i].x+=0.25*gx/n; path[i].y+=0.25*gy/n;
        }
        path=smoothTangent(path);
        if(!any) break;
    }
}

// ---------------- TCP helpers ----------------
static bool sendAll(int s,const std::string& d){ size_t n=0; while(n<d.size()){ ssize_t k=write(s,d.data()+n,d.size()-n); if(k<=0) return false; n+=k; } return true; }

// ---------------- main ----------------
int main(int argc,char** argv){
    std::cout.setf(std::ios::unitbuf);
    std::string ip="127.0.0.1"; int port=8091;
    if(argc>1) ip=argv[1]; if(argc>2) port=std::atoi(argv[2]);
    std::cout<<"Candidate solver connecting "<<ip<<":"<<port<<"\n";
    int sock=socket(AF_INET,SOCK_STREAM,0);
    sockaddr_in a{}; a.sin_family=AF_INET; a.sin_port=htons(port);
    inet_pton(AF_INET,ip.c_str(),&a.sin_addr);
    if(connect(sock,(sockaddr*)&a,sizeof(a))<0){ std::cerr<<"connect failed\n"; return 1; }
    std::cout<<"Connected\n";
    sendAll(sock,"Q\n");
    // robust recv of CONFIG+GRID
    std::string all; char tmp[65536];
    Pose2D start,goal; VehicleParams vp; int cols=0,rows=0;
    double map_w=100,map_h=100,res=0.2,ox=-50,oy=-50;
    std::vector<uint8_t> grid;
    for(int tries=0;tries<60;tries++){
        ssize_t b=read(sock,tmp,sizeof(tmp));
        if(b<=0){ std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }
        all.append(tmp,b);
        // try parse CONFIG
        auto pc=all.find("CONFIG");
        if(pc!=std::string::npos){ auto e=all.find('\n',pc); if(e!=std::string::npos){
            std::istringstream ss(all.substr(pc+7,e-pc-7));
            ss>>start.x>>start.y>>start.yaw>>goal.x>>goal.y>>goal.yaw
              >>vp.length>>vp.width>>vp.wheelbase>>vp.max_steer>>vp.max_speed>>vp.min_speed
              >>map_w>>map_h>>res>>ox>>oy>>cols>>rows;
        }}
        if(cols>0&&rows>0){
            auto pg=all.find("GRID");
            if(pg!=std::string::npos){
                // count numbers
                std::istringstream ss(all.substr(pg+4));
                size_t cnt; ss>>cnt;
                std::vector<uint8_t> g; g.reserve(cols*rows);
                int v; while((int)g.size()<cols*rows && (ss>>v)) g.push_back(v?1:0);
                if((int)g.size()>=cols*rows){ grid=std::move(g); break; }
            }
        }
    }
    if(grid.empty()||cols==0){ std::cerr<<"failed to load map, got "<<all.size()<<" bytes\n"; return 1; }
    std::cout<<"Map "<<cols<<"x"<<rows<<" res "<<res<<" start "<<start.x<<","<<start.y<<","<<start.yaw
             <<" goal "<<goal.x<<","<<goal.y<<","<<goal.yaw<<"\n";
    GridMap m; m.cols=cols;m.rows=rows;m.res=res;m.ox=ox;m.oy=oy;m.occ=grid; m.buildDist();
    std::cout<<"Dist map built\n";
    std::cout<<"probe car(-15.5,-8) occ="<<m.occWorld(-15.5,-8)<<" dist="<<m.distWorld(-15.5,-8)<<"\n";
    std::cout<<"probe graze(-14.45,-5.54) occ="<<m.occWorld(-14.45,-5.54)<<" dist="<<m.distWorld(-14.45,-5.54)<<"\n";
    std::cout<<"probe gap(-12,-8) occ="<<m.occWorld(-12,-8)<<" dist="<<m.distWorld(-12,-8)<<"\n";
    // classify
    auto isNear=[](double a,double b){return std::abs(a-b)<1.5;};
    int scen=2;
    if(isNear(start.x,-16)&&isNear(start.y,6)) scen=0;
    else if(isNear(start.x,-25)&&isNear(start.y,12)) scen=1;
    else if(isNear(start.x,-40)&&isNear(start.y,-40)){
        // Sc2 vs Sc3: probe wall cell (-10,0): Sc3 has 40x3 wall -> occupied; Sc2 free
        scen = m.occWorld(-10,0) ? 3 : 2;
    }
    std::cout<<"Scenario class="<<scen<<"\n";
    std::vector<Pose2D> path;
    if(scen==0||scen==1){
        int ggx,ggy; m.worldToGrid(goal.x,goal.y,ggx,ggy);
        auto h=dijkstraFromGoal(m,ggx,ggy,0.35);
        double clr = (scen==1?0.35:0.12);
        auto t0=std::chrono::steady_clock::now();
        path=hybridAStar(start,goal,m,vp,h,true,clr);
        auto t1=std::chrono::steady_clock::now();
        std::cout<<"Hybrid plan ms="<<std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count()<<"\n";
        if(path.empty()){ std::cerr<<"hybrid failed, fallback straight\n";
            for(int i=0;i<=60;i++){double t=(double)i/60; Pose2D q{start.x+(goal.x-start.x)*t,start.y+(goal.y-start.y)*t,normAng(start.yaw+normAng(goal.yaw-start.yaw)*t),0.5,0,1}; path.push_back(q);} }
        assignSpeeds(path,vp,0.9);
    } else {
        // 2D dijkstra legs
        std::vector<std::pair<double,double>> wps;
        std::vector<double>wyaw;
        if(scen==2){ wps={{goal.x,goal.y}}; wyaw={goal.yaw}; }
        else { wps={ {-30,20},{0,35},{30,25},{35,-15},{10,-35},{-25,-20},{0,0},{40,40} }; wyaw={0,0,0,0,0,0,0,goal.yaw}; }
        double cx=start.x, cy=start.y;
        std::vector<Pose2D> full;
        double infl=1.15;
        for(size_t L=0;L<wps.size();L++){
            int ggx,ggy; m.worldToGrid(wps[L].first,wps[L].second,ggx,ggy);
            auto h=dijkstraFromGoal(m,ggx,ggy,infl);
            auto leg=descentPath(m,h,cx,cy,wps[L].first,wps[L].second);
            leg=downsample(leg,0.5);
            // drop duplicate joint
            size_t s0 = full.empty()?0:1;
            for(size_t i=s0;i<leg.size();i++) full.push_back(leg[i]);
            cx=wps[L].first; cy=wps[L].second;
        }
        full=smoothTangent(downsample(full,0.6));
        // collision-safe shortcut smoothing (keep yaw feasible approx)
        // light moving-average with recheck
        for(int it=0;it<2;it++){
            auto sm=full;
            for(size_t i=2;i+2<sm.size();i++){ sm[i].x=(full[i-2].x+full[i-1].x+full[i].x+full[i+1].x+full[i+2].x)/5.0;
                sm[i].y=(full[i-2].y+full[i-1].y+full[i].y+full[i+1].y+full[i+2].y)/5.0; }
            sm=smoothTangent(sm);
            bool ok=true;
            for(size_t i=0;i<sm.size();i++) if(collideAt(sm[i].x,sm[i].y,sm[i].yaw,m,vp,0.35)){ok=false;break;}
            if(ok) full=sm; else break;
        }
        // blend final yaw to goal
        for(size_t i=0;i<full.size();i++){ double t=(double)i/full.size();
            if(t>0.93){ double b=(t-0.93)/0.07; full[i].yaw=normAng(full[i].yaw*(1-b)+goal.yaw*b); } }
        for(auto& q:full) q.dir=1;
        repairFootprint(full,m,vp);
        { int bad=0; for(auto& q:full) if(collideAt(q.x,q.y,q.yaw,m,vp,0.05)) bad++;
          std::cout<<"Footprint violations after repair: "<<bad<<"/"<<full.size()<<"\n"; }
        path=full;
        assignSpeeds(path,vp,1.9);
    }
    std::cout<<"Planned pts="<<path.size()<<"\n";
    { std::cout<<"Path tail (last 30):\n"; size_t s0=path.size()>30?path.size()-30:0;
      for(size_t i=s0;i<path.size();i++) std::cout<<"  i="<<i<<" "<<path[i].x<<","<<path[i].y<<","<<path[i].yaw<<" dir="<<path[i].dir<<" v="<<path[i].v<<"\n"; }
    { FILE* f=fopen("/tmp/planned_path.csv","w"); for(auto& q:path) fprintf(f,"%.3f,%.3f,%.3f,%d,%.2f\n",q.x,q.y,q.yaw,q.dir,q.v); fclose(f); }
    // upload TRAJ
    {
        std::ostringstream ss; ss<<"TRAJ ";
        for(size_t i=0;i<path.size();i+=2){ ss<<path[i].x<<" "<<path[i].y<<" "<<path[i].yaw<<" "<<path[i].v<<";"; }
        ss<<"\n"; sendAll(sock,ss.str());
        std::cout<<"TRAJ uploaded "<<ss.str().size()<<" bytes\n";
    }
    // ---- closed-loop tracking (pure pursuit fwd / heading-P rev + docking) ----
    std::string leftover;
    int curFloor=0; // monotonic floor so cusp-skips stick
    auto nearestAhead=[&](double x,double y,int from)->int{
        int best=from; double bd=1e18;
        int lo=std::max({0,from-10,curFloor-3});
        int hi=std::min((int)path.size(),from+400);
        for(int i=lo;i<hi;i++){ double d=std::hypot(path[i].x-x,path[i].y-y); if(d<bd){bd=d;best=i;} }
        return best;
    };
    int cur=0, cuspHold=0, settle=0, dockDir=0;
    double lastDelta=0;
    while(true){
        char b[8192]; ssize_t n=read(sock,b,sizeof(b)-1);
        if(n<=0){ std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }
        b[n]=0; leftover.append(b,n);
        // extract last complete TELEMETRY line
        std::string lastLine; size_t pos;
        while((pos=leftover.find('\n'))!=std::string::npos){ lastLine=leftover.substr(0,pos); leftover.erase(0,pos+1); }
        if(lastLine.find("TELEMETRY")==std::string::npos) continue;
        std::istringstream ts(lastLine);
        std::string tag; uint64_t step; double tms,cx,cy,cyaw,cv,cd; int coll,done;
        ts>>tag>>step>>tms>>cx>>cy>>cyaw>>cv>>cd>>coll>>done;
        if(coll){ std::cout<<"Collision at "<<cx<<","<<cy<<","<<cyaw<<" dist="<<m.distWorld(cx,cy)<<" occ="<<m.occWorld(cx,cy)<<"\n"; break; }
        if(done){ std::cout<<"Goal reached!\n"; break; }
        if(step>6000){ std::cout<<"step limit\n"; sendAll(sock,"CTRL 0 0\n"); break; }
        cur=nearestAhead(cx,cy,cur);
        curFloor=std::max(curFloor,cur-5);
        double dg=std::hypot(goal.x-cx,goal.y-cy);
        double cmdV, cmdD;
        if(dg>1.5) dockDir=0; // allow fresh pick if we leave the dock zone
        if(dg<0.45||(cur>=(int)path.size()-2&&dg<1.2)){
            // final docking with latched direction + early stop inside tolerance
            if(dg<0.36&&angDiff(cyaw,goal.yaw)<0.26){ sendAll(sock,"CTRL 0 0\n"); continue; }
            double bearing=std::atan2(goal.y-cy,goal.x-cx);
            double ef=normAng(bearing-cyaw), er=normAng(bearing-cyaw-M_PI);
            if(dockDir==0) dockDir=(std::abs(ef)<=std::abs(er))?1:-1;
            else if(dockDir==1&&std::abs(er)<std::abs(ef)-0.5) dockDir=-1;
            else if(dockDir==-1&&std::abs(ef)<std::abs(er)-0.5) dockDir=1;
            double e=(dockDir>0)?ef:er;
            // close-in: blend toward goal yaw so we arrive aligned
            if(dg<0.55){ double ey=normAng(goal.yaw-cyaw)*(dockDir>0?1.0:-1.0);
                // for reverse, yaw decreases with left steer; blend carefully:
                e = (dockDir>0)? (0.5*e+0.5*normAng(goal.yaw-cyaw)) : (0.6*e-0.4*normAng(goal.yaw-cyaw)); }
            cmdD=clampD(1.8*e,-vp.max_steer,vp.max_steer);
            cmdV=clampD(0.5*dg,-0.22,0.22)*(dockDir>0?1:-1);
            if(dg>0.15&&std::abs(cmdV)<0.1) cmdV=(dockDir>0?0.12:-0.12);
        } else {
            int look=cur;
            bool parking = (scen==0||scen==1);
            double Ld = parking ? clampD(1.0+0.4*std::abs(cv),1.0,1.6)
                                : clampD(1.6+0.7*std::abs(cv),1.4,3.2);
            if(!parking&&dg<4.0) Ld=std::min(Ld,1.3); // tight endgame for yaw alignment
            double acc=0;
            while(look+1<(int)path.size()){ acc+=std::hypot(path[look+1].x-path[look].x,path[look+1].y-path[look].y); look++; if(acc>=Ld) break; }
            // cusp handling: direction flip within lookahead -> creep to cusp, stop, skip past
            int cuspIdx=-1;
            for(int i=cur;i<=look&&i<(int)path.size();i++) if(path[i].dir!=path[cur].dir){ cuspIdx=i; break; }
            int dirS=path[cur].dir;
            if(cuspIdx>=0){
                double dc=std::hypot(path[cuspIdx].x-cx,path[cuspIdx].y-cy);
                if(dc>0.6){
                    look=cuspIdx;
                    double tx=path[look].x, ty=path[look].y;
                    double e = (dirS>0)? normAng(std::atan2(ty-cy,tx-cx)-cyaw)
                                       : normAng(std::atan2(ty-cy,tx-cx)-cyaw-M_PI);
                    cmdD=clampD(2.0*e,-vp.max_steer,vp.max_steer);
                    cmdV=0.4*dirS;
                    cuspHold=0;
                } else { cmdV=0; cmdD=lastDelta; if(++cuspHold>15){ cur=std::min(cuspIdx+2,(int)path.size()-1); curFloor=cur; cuspHold=0; } }
            }
            else {
                cuspHold=0;
                // slow down when close to obstacles (body-aware tracking margin)
                bool parking2=(scen==0||scen==1);
                double dd_=bodyMinDist(cx,cy,cyaw,m,vp);
                double slowF= parking2 ? clampD((dd_-0.20)/0.6,0.25,1.0)
                                       : clampD((dd_-0.30)/0.9,0.25,1.0);
                double tx=path[look].x, ty=path[look].y;
                if(dirS<0){
                    // reverse: heading-P referenced to rear axis, tight lookahead
                    double rLd = (dg<1.5)?0.8:1.2;
                    int rlook=cur; double racc=0;
                    while(rlook+1<(int)path.size()){ racc+=std::hypot(path[rlook+1].x-path[rlook].x,path[rlook+1].y-path[rlook].y); rlook++; if(racc>=rLd) break; }
                    double e=normAng(std::atan2(path[rlook].y-cy,path[rlook].x-cx)-cyaw-M_PI);
                    cmdD=clampD(2.0*e,-vp.max_steer,vp.max_steer);
                    double pv=path[cur].v!=0?path[cur].v:-0.6;
                    cmdV=clampD(pv,-0.6,-0.15)*slowF;
                    if(std::abs(cmdV)<0.15) cmdV=-0.15;
                } else {
                    double alpha=normAng(std::atan2(ty-cy,tx-cx)-cyaw);
                    double R=Ld/(2*std::sin(alpha)+1e-9);
                    double dd=clampD(std::atan(vp.wheelbase/R),-vp.max_steer,vp.max_steer);
                    cmdD=dd;
                    double pv=path[cur].v!=0?path[cur].v:1.0;
                    cmdV=clampD(pv,-1.6,2.2)*slowF;
                    // slow in curves
                    cmdV*=clampD(1.0-std::abs(cmdD)/vp.max_steer*0.5,0.45,1.0);
                    if(!parking&&dg<4.0) cmdV=std::min(cmdV,0.9); // nav endgame
                    if(dirS>0&&std::abs(cmdV)<0.15&&cur<(int)path.size()-3) cmdV=0.12;
                }
            }
        }
        // steer rate limit on command side
        double mx=vp.max_steer_rate*0.06;
        cmdD=lastDelta+clampD(cmdD-lastDelta,-mx,mx);
        lastDelta=cmdD;
        if(step%25==0) std::cout<<"st="<<step<<" pos="<<cx<<","<<cy<<","<<cyaw<<" cur="<<cur<<"/"<<path.size()<<" dg="<<dg<<" cmd="<<cmdV<<","<<cmdD<<"\n";
        std::ostringstream cs; cs<<"CTRL "<<cmdV<<" "<<cmdD<<"\n";
        if(!sendAll(sock,cs.str())) break;
    }
    close(sock); return 0;
}
