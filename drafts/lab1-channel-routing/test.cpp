
#include <vector>
#include<algorithm>
#include <queue>
#include <iostream>
#include <utility>
#include <climits>

using tracks = std::vector<int>;


#include <map>

#define startTrack 1
//cause we only can get the realy channel width after the program end,we need to save Info for TopEdge,BotEdge.
#define BotEdge 0
#define TopEdge INT_MAX
#define SteadyNetCstr 3 
#define MinJogLen 2

struct channel{
    channel(int initWidth){
        tks.resize(initWidth+1,0);//0 is ignore.
    }
    tracks tks;//from index : start from 1 to real Channel Width;
    std::vector<int>topEdge;
    std::vector<int>botEdge;
    int queryTop(int col);//return 0 if no such tracks.
    int queryBot(int col);
    int totalColumn;
    int Width(){return tks.size()-1;}
    void clear_if(int netid);
};
int channel::queryTop(int col){
    if(topEdge.at(col)==0)return TopEdge;
    for(int idx = tks.size()-1;idx >= startTrack;idx--)
        if(tks.at(idx)==topEdge.at(col)||tks.at(idx)==0)
            return idx;
    return TopEdge;
}
int channel::queryBot(int col){
    if(botEdge.at(col)==0)return BotEdge;
    for(int idx = startTrack;idx < tks.size();idx++)
        if(tks.at(idx)==botEdge.at(col)||tks.at(idx)==0)
            return idx;
    return BotEdge;
}
void channel::clear_if(int netid){
    for(int i = 1;i < tks.size();i++){
        if(tks.at(i)==netid)
            tks.at(i) = 0;
    }
}

void showTrack(channel&C){
    for(int i = 1;i<C.tks.size();i++)
        if(C.tks.at(i))
            std::cout<<C.tks.at(i)<<" at track:"<<i<<"\n";
}
struct segment{
    int col,track,target;
};
struct Net{
    int netId;
    std::queue<int>topPins;
    std::queue<int>botPins;
    std::vector<segment>HSegment;
    std::vector<segment>VSegment;
    // Call whenever bringIn to existed track or doing collapse/shrinking range.
    void AddVS(int col,int track,int ttrack){
        if(track>ttrack)std::swap(track,ttrack);
        VSegment.push_back({col,track,ttrack});
    }
    void AddHS(int col,int track,int tcol){
        if(col>tcol)std::swap(col,tcol);
        HSegment.push_back({col,track,tcol});
    }

    std::pair<int,int>GetNextPins(){
        int pin1 = (topPins.empty()) ? -1 : topPins.front();
        int pin2 = (botPins.empty()) ? -1 : botPins.front();
        return {pin1,pin2};
    }

    //<track,startCol>
    std::map<int,int>uti_track;
    bool isDone(){
        return topPins.empty() && botPins.empty() && uti_track.empty();
    }
    bool ConnectToTrack(int col,int track,bool topPin){
        if(topPin){
            AddVS(col,track,TopEdge);
            topPins.pop();
        }else{
            AddVS(col,BotEdge,track);
            botPins.pop();
        }
        //update uti_track
        auto pos = uti_track.find(track);
        if(pos==uti_track.end())//new track
            pos = uti_track.insert(pos,{track,col});
        //this net is done.
        //only one track and no more pins.
        if(lastTrack()){
            AddHS(pos->second,track,col);
            uti_track.clear();
            return true;
        }
        return false;
    }
    void TopBotConnect(int col){
        topPins.pop();
        botPins.pop();
        AddVS(col,BotEdge,TopEdge);
        for(auto uti:uti_track)AddHS(uti.second,uti.first,col);
        uti_track.clear();
    }
    bool collapse(int col,int t1,int t2){
        auto origin = uti_track.find(t1);
        auto target = uti_track.find(t2);
        if(origin==uti_track.end()||target==uti_track.end()){
            std::cerr<<"collapse error! ";
            if(origin==uti_track.end())std::cerr<<netId<<" not occupy"<<t1<<"\n";
            else std::cerr<<netId<<" not occupy"<<t2<<"\n";
            exit(1);
        }
        if(col!=origin->second)
        AddHS(origin->second,t1,col);
        AddVS(col,t1,t2);
        uti_track.erase(origin);
        if(lastTrack()){
            if(col!=target->second)
            AddHS(target->second,t2,col);
            uti_track.clear();
            return true;
        }
        return false;
    }
    void move(int col,int t1,int t2){
        auto origin = uti_track.find(t1);
        if(origin==uti_track.end()){
            std::cerr<<"move error! "<<netId<<" not occupy"<<t1<<"\n";
            exit(1);
        }
        if(col!=origin->second)
        AddHS(origin->second,t1,col);
        AddVS(col,t1,t2);

        uti_track.erase(origin);
        uti_track.insert({t2,col});
    }
    bool lastTrack(){
        return uti_track.size()==1 && topPins.empty() && botPins.empty();
    }
};

using interval = std::pair<int,int>;
struct Cstr{
    std::vector<interval>intervals;//first < second .
    bool overlapCstr(int t1,int t2){
        if(t1>t2)std::swap(t1,t2);
        for(auto interval:intervals){
            if(! (t1 >= interval.second || t2 <= interval.first) )
                return true;
        }
        return false;
    }
    void newCstr(int t1,int t2){
        if(t1 > t2)std::swap(t1,t2);
        intervals.push_back({t1,t2});
    }
};

//used for collapse , shrink
//after doing this, t1 is free,but still need some hint for constraint to avoid using t1 in this stage.
void collapse(channel&C,Net&net,int col,int t1,int t2,std::vector<int>&History){
    History.at(t1) = t2;
    C.tks.at(t1) = 0;
    if(net.collapse(col,t1,t2))C.tks.at(t2) = 0;
    else C.tks.at(t2) = net.netId;
}
void move(channel&C,Net&net,int col,int t1,int t2){
    if(C.tks.at(t2)!=net.netId &&C.tks.at(t2)!=0){
        std::cerr<<"move error,track"<<t2<<"is not empty!\n";
        exit(1);
    }
    C.tks.at(t1) = 0;
    net.move(col,t1,t2);
    C.tks.at(t2) = net.netId;
}



void bringIn(channel&C,int col,int track,Net&net,bool TopPin){
    if(net.netId==0)return;//ignore.
    if(C.tks.at(track)!=0 && C.tks.at(track)!=net.netId){//this track is invalid.
        std::cerr<<"place non-empty track!\n";exit(1);
    }
    if(!net.ConnectToTrack(col,track,TopPin))//net not done
        C.tks.at(track) = net.netId;//setting netId
    else 
        C.tks.at(track) = 0;//free track
}



//called by stepA:Bring in the pin to the track and try jump more.
//only called by there is only one side pins after col
void escapeJump(bool Rising,Net&net,channel&C,int col,int*track,int illegal){
    auto pins = net.GetNextPins();
    int topPinX = pins.first;
    int botPinX = pins.second;
    if(Rising&&botPinX!=-1)return;
    if(Rising&&topPinX==-1)return;
    if(!Rising&&topPinX!=-1)return;
    if(!Rising&&botPinX==-1)return;

    int up = Rising ? 1 : -1;
    int t = *track + up;
    for(;t != C.tks.size() && t!=0 &&t!=illegal;t+=up){
        if((!C.tks.at(t)||C.tks.at(t)==net.netId))//find the t we want
            break;
    }
    if(t!=0&&t!=C.tks.size()&&t!=illegal){
        move(C,net,col,*track,t);
        *track = t;
    }
}

#define Complete 0
#define Top 1
#define Bot 2
#define Non 3

//StepA : bring in the pins!
int stepA(channel&C,int col,std::vector<Net>&Nets,Cstr&cstr,bool escape=false){
    int track1 = C.queryTop(col);//if Pin = zero,get *track1 = TopEdge.
    int track2 = C.queryBot(col);//if Pin = zero,get *track2 = BotEdge.
    int topPin = C.topEdge.at(col);
    int botPin = C.botEdge.at(col);
    if(track1==TopEdge && track2==BotEdge)return Non;// neither top nor bot is done.
    int choose;
    if(track1!=TopEdge && track2!=BotEdge){//Both pins have valid track.
        if(track1 > track2 || topPin == botPin)// no conflict
            choose = Complete;
        else//choose smallest length one.
            choose = (C.tks.size() - track1 <= track2) ? Top:Bot;
    }
    else{
        choose = (track1==TopEdge)? Bot:Top;
    }
    if(choose==Complete){
        bringIn(C,col,track1,Nets.at(topPin),true);
        bringIn(C,col,track2,Nets.at(botPin),false);
        if(escape){
            escapeJump(true,Nets.at(botPin),C,col,&track2,track1);
            escapeJump(false,Nets.at(topPin),C,col,&track1,track2);
        }
        cstr.newCstr(track1,TopEdge);
        cstr.newCstr(BotEdge,track2);

    }else if(choose==Top){
        bringIn(C,col,track1,Nets.at(topPin),true);
        if(escape){
            escapeJump(false,Nets.at(topPin),C,col,&track1,track2);
        }
        cstr.newCstr(track1,TopEdge);
        if(botPin==0)choose = Complete;
    }else{
        bringIn(C,col,track2,Nets.at(botPin),false);
        if(escape){
            escapeJump(true,Nets.at(botPin),C,col,&track2,track1);
        }
        cstr.newCstr(BotEdge,track2);
        if(topPin==0)choose = Complete;
    }
    return choose;
}

struct collapsejog{
    int t1,t2,netid;
};

//Search all possible collapse jogs not overlap with cstr.
std::vector<collapsejog> collapseJogs(channel&C,Cstr&cstr){
    std::vector<collapsejog>candidates;//first:netid,second:last track
    std::map<int,int>netRecord;
    for(int i = 1;i<C.tks.size();i++){
        int netid = C.tks.at(i);
        if(netid){//non-empty
            auto lastTrack = netRecord.find(netid);
            if(lastTrack==netRecord.end()){
                netRecord.insert(lastTrack,{netid,i});
            }else if(!cstr.overlapCstr(lastTrack->second,i)){//do not overlap with cstr.
                candidates.push_back({lastTrack->second,i,netid});
                lastTrack->second = i;//update last
            }
        }
    }
    std::sort(candidates.begin(),candidates.end(),[](collapsejog&lhs,collapsejog&rhs){return lhs.t1<rhs.t1;});//sorted by t1
    return candidates;
}

// return the best Candidates idx
// consider : closest track ,second closest track,total length to break tie.
int BreakTie(std::vector<collapsejog>&jogs,std::vector<std::pair<int,int>>&tieInfo,std::vector<int>&Candidates,int channelWidth){
    int mostOuter1 = INT_MAX,mostOuter2=INT_MAX;
    int longest = 0,best = 0;
    auto replace = [&mostOuter1,&mostOuter2,&longest,&best](int close1,int close2,int len,int idx){mostOuter1 = close1;mostOuter2 = close2;longest = len;best = idx;};
    for(int i = 0;i < Candidates.size();i++){
        int DPidx = Candidates.at(i);
        int upperTrack = jogs[DPidx].t2;
        int lowerTrack = jogs[tieInfo[DPidx].first].t1;

        //three info we need to consider 
        int totalLen = tieInfo[DPidx].second;
        int close1 = lowerTrack;
        int close2 = channelWidth - upperTrack + 1;
        if(close1 > close2)std::swap(close1,close2);

        if(close1 < mostOuter1){
            replace(close1,close2,totalLen,i);
        }else if(close1 == mostOuter1){
            if(close2 < mostOuter2){
                replace(close1,close2,totalLen,i);
            }else if(close2==mostOuter2 && totalLen < longest){
                replace(close1,close2,totalLen,i);
            }
        }
    }
    return best;
}

//return the jogs's index which composes the optimal solution.
std::vector<int> ActivitySelection(std::vector<collapsejog>&jogs,int channelWidth){

    //DP[i] = i : means select jogs[i] is the maximal solution when consider jogs[i] is the last activity.
    //DP[i] = j,j < i means the Set : (jogs[i] union the set DP[j] represent )is the maximal solution when consider jogs[i] is the last activity.

    //DP[i].first : maxNumber  when consider jogs[i] is the last activity.
    //DP[i].second : if== i then: DP[i] represent jogs[i], otherwise DP[i] represent (the set jogs[i] union the set DP[DP[i]] represent).  
    std::vector<std::pair<int,int>>DP(jogs.size(),{-1,-1});//DP
    std::vector<std::pair<int,int>>tieInfo(jogs.size(),{0,0});//lowest jog index,longest length

    for(int i = 0;i < jogs.size();i++){
        int net1 = jogs.at(i).netid;
        int jogLen = jogs.at(i).t2 - jogs.at(i).t1; 
        //initial 
        DP[i].first = 1;//number : only jogs[i]
        DP[i].second = i;//i
        tieInfo[i].first = i;
        tieInfo[i].second = jogLen; 
        for(int j = 0;j < i;j++){//scan from front.
            int net2 = jogs.at(j).netid; 
            int endTrack = jogs.at(j).t2;
            if(net1==net2 || endTrack < jogs.at(i).t1){//non-conflict
                if(DP[j].first + 1 > DP[i].first){
                    DP[i].first = DP[j].first + 1;
                    DP[i].second = j;
                    tieInfo[i].first = tieInfo[j].first;
                    tieInfo[i].second = tieInfo[j].second + jogLen;
                }else if(DP[j].first+1 == DP[i].first){
                    //reserve the most lower,most long one.
                    if(tieInfo[j].first < tieInfo[i].first){//lower than now
                        DP[i].second = j;
                        tieInfo[i].first = tieInfo[j].first;
                        tieInfo[i].second = tieInfo[j].second + jogLen;
                    }else if(tieInfo[j].first==tieInfo[i].first && tieInfo[j].second + jogLen > tieInfo[i].second){
                        DP[i].second = j;
                        tieInfo[i].first = tieInfo[j].first;
                        tieInfo[i].second = tieInfo[j].second + jogLen;
                    }
                }
            }
        }
    }


    //selection the DP which has maximum #activities.
    std::vector<int>Candidates;//save the jog's index.
    int maxN = 0;
    for(int i = 0;i<jogs.size();i++){
        if(DP[i].first==maxN)Candidates.push_back(i);
        else if(DP[i].first > maxN){
            Candidates.clear();
            Candidates.push_back(i);
            maxN = DP[i].first;
        }
    }

// DEBUF Info
/*
    for(int i = 0;i<Candidates.size();i++){
        std::cout<<"candidate:"<<i<<"\n";
        int c = Candidates.at(i);
        std::cout<<jogs.at(c).t1<<" "<<jogs.at(c).t2<<"\n";
        while(c!=DP[c].second){
            c = DP[c].second;
            std::cout<<jogs.at(c).t1<<" "<<jogs.at(c).t2<<"\n";
        }
    }
*/
    int bestc = BreakTie(jogs,tieInfo,Candidates,channelWidth);
    std::vector<int>jogSolution;
    int DPidx = Candidates[bestc];
    while(DP[DPidx].second != DPidx){
        jogSolution.push_back(DPidx);
        DPidx = DP[DPidx].second;
    }
    jogSolution.push_back(DPidx);
    return jogSolution;
}

void showPattern(std::vector<int>PatternIdx,std::vector<collapsejog>&jogs){
    for(auto idx:PatternIdx){
        std::cout<<jogs.at(idx).t2<<" "<<jogs.at(idx).t1<<"\n";
    }
}

//stepB : free up the tracks!
void stepB(channel&C,int col,std::vector<Net>&nets,Cstr&cstr,bool escape=false){
    auto collapsejogs = collapseJogs(C,cstr);//according to cstr,generate all legal jogs.
    if(collapsejogs.empty())return;
    auto pattern = ActivitySelection(collapsejogs,C.Width());//find the optimal pattern.
    std::vector<int>History(C.tks.size(),0);
    for(int i = 1;i<History.size();i++)History.at(i)=i;
    //do free up 
    for(auto idx:pattern){//index is from top to down ....
        int netid = collapsejogs.at(idx).netid;
        int t1 = History[collapsejogs.at(idx).t1];
        int t2 = History[collapsejogs.at(idx).t2];
        auto pins = nets.at(netid).GetNextPins();
        if(escape)
        collapse(C,nets.at(netid),col,t2,t1,History);
        else
        collapse(C,nets.at(netid),col,t1,t2,History);
    }
    for(auto idx:pattern)
        cstr.newCstr(collapsejogs.at(idx).t1,collapsejogs.at(idx).t2);
}



#define CantStay 1
#define CantCross 2 //can't stay too
std::vector<int> moveCstr(channel&C,Cstr&cstr){
    std::vector<int> trackMark(C.tks.size(),0);//0 is can stay and cross. 
    for(int i = 0;i<C.tks.size();i++)
        if(C.tks[i])trackMark[i] = CantStay;
    for(auto cst:cstr.intervals){
        int t1 = cst.first;
        int t2 = (cst.second==TopEdge)? C.tks.size()-1:cst.second;
        for(int i = t1;i<=t2;i++)trackMark.at(i) = CantCross;
    }
    return trackMark;
}

bool Can_F_Or_R(Net&net){
    if(net.uti_track.size()==1){
        auto pins = net.GetNextPins();
        if(pins.first==-1||pins.second==-1||std::abs(pins.first-pins.second)>=SteadyNetCstr)
            return true;
    }
    return false;
}

std::vector<int>moveCandidates(channel&C,std::vector<Net>&nets){
    std::vector<int>Candidates;
    for(auto netid:C.tks){
        if(netid&&Can_F_Or_R(nets.at(netid)))
            Candidates.push_back(netid);
    }
    std::sort(Candidates.begin(),Candidates.end(),[&nets](int lhs,int rhs){
        auto pins1 = nets.at(lhs).GetNextPins();
        auto pins2 = nets.at(rhs).GetNextPins();
        int closestPin1 = pins1.first==-1 ?  pins1.second : (pins1.second ==-1 ? pins1.first : std::min(pins1.first,pins1.second)); 
        int closestPin2 = pins2.first==-1 ?  pins2.second : (pins2.second ==-1 ? pins2.first : std::min(pins2.first,pins2.second)); 
        return closestPin1 < closestPin2;
    });
    return Candidates;
}


int Jog(bool Rising,int track,const std::vector<int>&trackMark,bool limit=false){
    int up = Rising ? 1:-1;
    int track2 = track;
    for(int t = track+up;t!=trackMark.size()&&t!=0;t+=up){
        if(trackMark.at(t)==CantCross)break;
        if(trackMark.at(t)==CantStay)continue;
        /*if(limit){
        if(track < trackMark.size() /2 && track2 > trackMark.size()/2)break;
        if(track > trackMark.size() /2 && track2 < trackMark.size()/2)break;
        }*/
        track2 = t;
    }
    return track2;
}


void Dojog(channel&C,int col,Net&net,int track,bool Rising,std::vector<int>&trackMark,Cstr&cstr){
    int track2 = Jog(Rising,track,trackMark);//get track2
    if(std::abs(track-track2) >= MinJogLen){
        move(C,net,col,track,track2);
        //update trackMark
        if(track > track2)std::swap(track,track2);
        for(int t = track;t<=track2;t++)
            trackMark.at(t) = CantCross;
        cstr.newCstr(track,track2);
    }
}

void stepC(channel&C,int col,std::vector<Net>&nets,Cstr&cstr){

    //scan track to find uncombine jog
    std::map<int,int>netInfo;
    for(auto tk:C.tks){
        if(tk){
            auto find = netInfo.find(tk);
            if(find==netInfo.end()){
                netInfo.insert(find,{tk,1});
            }else 
                find->second++;
        }
    }
    std::vector<int>Candidates;
    for(auto info:netInfo){
        if(info.second==2)
            Candidates.push_back(info.first);
    }
    if(Candidates.empty())return ;

    //if find,do shrink , sorted by large jog to small jog
    std::sort(Candidates.begin(),Candidates.end(),[&nets](int lhs,int rhs)
        {
            Net&net1 = nets.at(lhs);
            Net&net2 = nets.at(rhs);
            auto tracks1 = net1.uti_track;
            auto tracks2 = net2.uti_track;
            int t11 = tracks1.begin()->first;
            int t12 = tracks1.rbegin()->first;
            int t21 = tracks2.begin()->first;
            int t22 = tracks2.rbegin()->first;
            return std::abs(t11-t12) < std::abs(t21-t22);
        }
    ); 
    std::vector<int> trackMark = moveCstr(C,cstr);//get MoveCstr
    //process from largest jog and unpdate trackMark
    for(auto netid:Candidates){
        Net&net = nets.at(netid);
        int trackup = net.uti_track.begin()->first;
        int trackdown = net.uti_track.rbegin()->first;
        if(trackup < trackdown)std::swap(trackup,trackdown);

        Dojog(C,col,net,trackup,false,trackMark,cstr);
        Dojog(C,col,net,trackdown,true,trackMark,cstr);

    }




    //update cstr
}

std::vector<int> pinFirst(channel&C,int col,std::vector<Net>&nets){
    if(col>=C.totalColumn)return {};
    static bool botFirst = true;
    int topPin = C.topEdge.at(col);
    int botPin = C.botEdge.at(col);
    std::vector<int>candidates;
    if(topPin&&Can_F_Or_R(nets.at(topPin)))
        candidates.push_back(topPin);
    if(botPin&&Can_F_Or_R(nets.at(botPin)))
        candidates.push_back(botPin);
    if(candidates.size()==2){
        if(botFirst){
            std::swap(candidates.at(0),candidates.at(1));
            botFirst = false;
        }else{
            botFirst = true;
        }
    }
    return candidates;
}

//falling or rising
void stepD(channel&C,int col,std::vector<Net>&nets,Cstr&cstr){
    //auto candidates = moveCandidates(C,nets);//find the net which only occupy one track and achieve some condition.
    std::vector<int>candidates = pinFirst(C,col,nets);
    std::vector<int>c2 = moveCandidates(C,nets);
    for(auto c:c2)candidates.push_back(c);
    if(candidates.empty())return ;
    std::vector<int> trackMark = moveCstr(C,cstr);//get MoveCstr
    for(auto netid:candidates){
        Net&net = nets.at(netid);
        int track = net.uti_track.begin()->first;//get the track occupied by this net.
        auto pins = net.GetNextPins();
        bool Rising;
        if(pins.first==-1)Rising = false;
        else if(pins.second==-1)Rising = true;
        else if(pins.first < pins.second)Rising = true;
        else Rising = false;
        int track2 = Jog(Rising,track,trackMark,candidates.size()>2);//get track2
        if(std::abs(track-track2) >= MinJogLen){
            move(C,net,col,track,track2);
            //update trackMark
            if(track > track2)std::swap(track,track2);
            for(int t = track;t<=track2;t++)
                trackMark.at(t) = CantCross;
        }
    }
}


bool notDone(channel&C){
    bool flag = false;
    for(int i = 1 ;i < C.tks.size();i++){
        if(C.tks.at(i)!=0){
            flag = true;
        }
    }
    return flag;
}



void stepE(int errorNum){
    if(errorNum!=Complete){

        if(errorNum==Top){//botPin not done

        }else if(errorNum==Bot){//topPin not done

        }else{//both pin not done

        }
    }
}


void getPins(channel&C,int col,int*TopPin,int*BotPin){
    if(C.totalColumn > col){
        *TopPin = C.topEdge.at(col);
        *BotPin = C.botEdge.at(col);
    }else{
        *TopPin = 0,*BotPin = 0;
    }
}

bool lastPins(Net&net){
    return net.topPins.size()==1&&net.botPins.size()==1&&net.botPins.front()==net.topPins.front();
}
void ConnectTopBot(channel&C,Net&net){
    int col = net.botPins.front();
    net.TopBotConnect(col);
    C.clear_if(net.netId);
}


int GCR(channel &C,std::vector<Net>&Nets){ //Greedy-Channel-Route

    int col;
    for(col = 0;col < C.totalColumn || notDone(C);col++){
        int TopPin,BotPin;
        getPins(C,col,&TopPin,&BotPin);
        //connect directly Fig E.
        if(TopPin && lastPins(Nets.at(TopPin))){
            ConnectTopBot(C,Nets.at(TopPin));
        }
        else{
            Cstr cstr;
            int errorNum = 0;
            if(TopPin||BotPin){
                errorNum = stepA(C,col,Nets,cstr,col+10 > C.totalColumn);
            }
            stepB(C,col,Nets,cstr,col+6 > C.totalColumn);
            stepC(C,col,Nets,cstr);
            stepD(C,col,Nets,cstr);
            stepE(errorNum);
            if(errorNum){
                std::cout<<"col:"<<col<<"error:"<<errorNum<<"\n";
            }

            /*
            if(errorNum!=Complete){
                std::cerr<<"failed in col:"<<col<<"\n";exit(1);
            }*/
        }
    }
    return col-1;
}









#include <fstream>
#include <string>
void parser(channel&C,std::vector<Net>&nets,std::string fileName);
void output(int finalWidth,std::vector<Net>&nets,std::string fileName);

//for test stepB
void testAddTrack(int track,Net&net,channel&C){
    net.uti_track.insert({track,0});
    C.tks.at(track)=net.netId;
}

int main(int argc,char*argv[]){

    channel C(21);
    std::vector<Net>nets;

    parser(C,nets,argv[1]);
    int col = GCR(C,nets);
    output(C.Width(),nets,argv[2]);

    std::cout<<"total col:"<<C.totalColumn<<"\n";
    std::cout<<"use col:"<<col<<"\n";
    std::cout<<"track:"<<C.Width()<<"\n";


    return 0;
}

void removeOverlap(Net&net){

    //sorted by veritcal length 
    std::sort(net.VSegment.begin(),net.VSegment.end(),[](segment&s1,segment&s2){
        return s1.target-s1.target > s2.target-s2.track;
    });

    std::vector<segment>newVs;
    for(int i = 0;i<net.VSegment.size();i++){
        bool isoverlap = false;
        int t1 = net.VSegment.at(i).track;
        int t2 = net.VSegment.at(i).target;
        for(int j = 0;j<i;j++){
            if(net.VSegment.at(j).col!=net.VSegment.at(i).col)continue;
            int t3 = net.VSegment.at(j).track;
            int t4 = net.VSegment.at(j).target;
            if(t1>=t3&&t2<=t4)//Vs[i] is in Vs[j]
            {
                isoverlap = true;
                break;
            }
        }
        if(!isoverlap)
            newVs.push_back(net.VSegment.at(i));
    }
    net.VSegment = std::move(newVs);
}

void output(int finalWidth,std::vector<Net>&nets,std::string fileName){

    std::ofstream out{fileName};
    if(!out){std::cerr<<"can't open"<<fileName<<"\n";exit(1);}

    for(int i = 1;i<nets.size();i++){
        Net& n = nets.at(i);
        removeOverlap(n);
        if(n.VSegment.size()==0&&n.HSegment.size()==0)continue;
        out<<".begin "<<n.netId<<"\n";
        for(auto v:n.VSegment){
            int y1 = (v.track==TopEdge)? finalWidth+2 : v.track+1;
            int y2 = (v.target==TopEdge)? finalWidth+2 :v.target+1;
            if(y1>y2)std::swap(y1,y2);//make y1 < y2
            out<<".V "<<v.col<<" "<<y1<<" "<<y2<<"\n";
        }
        for(auto h:n.HSegment){
            int x1 = h.col;
            int x2 = h.target;
            if(x1>x2)std::swap(x1,x2);//make x1 < x2
            out<<".H "<<x1<<" "<<h.track+1<<" "<<x2<<"\n";
        }
        out<<".end\n";
    }

    out.close();
}

void parser(channel&C,std::vector<Net>&nets,std::string fileName){

    std::ifstream in{fileName};
    if(!in){std::cerr<<"can't open"<<fileName<<"\n";exit(1);}

    std::vector<int>line;


    //get pins.
    int n;
    while(in>>n)line.push_back(n);

    //allocate Channel
    C.topEdge.reserve(line.size()/2);
    C.botEdge.reserve(line.size()/2);
    C.totalColumn = line.size()/2;

    //allocate nets.
    int maxNet=0;
    for(auto n:line)if(n>maxNet)maxNet = n;
    nets.resize(maxNet+1);
    for(int i = 0;i<=maxNet;i++)nets.at(i).netId = i;


    for(int i = 0;i<line.size()/2;i++){
        C.topEdge.push_back(line.at(i));
        nets.at(line.at(i)).topPins.push(i);
    }
    for(int i = line.size()/2;i<line.size();i++){
        C.botEdge.push_back(line.at(i));
        nets.at(line.at(i)).botPins.push(i-line.size()/2);
    }
    in.close();
}
