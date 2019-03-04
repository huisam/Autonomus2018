class Initial{
    public:
        Initial():Exist(false),X(0),Y(0) {};
        ~Initial();
        bool testExist() { return Exist; };
        void input(double inputX, double inputY);
        void paraminput(bool param);
        double getX() {return X;};
        double getY() {return Y;};

    private:
        bool Exist;
        double X;
        double Y;
};

Initial::~Initial()
{}

void Initial::input(double inputX, double inputY)
{
    X = inputX;
    Y = inputY;
    Exist = true;
}
void Initial::paraminput(bool param)
{
    Exist = !param;
}

