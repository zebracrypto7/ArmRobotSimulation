class InverseKinematics {
    constructor(){

        this.dhd0 = 105;
        this.dhd1 = 165;
        this.dhd2 = 0;
        this.dhd3 = 0;
        this.dhd4 = 400;
        this.dhd5 = 0;
        this.dhd6 = 90;

        this.dha2 = 200;
        this.dha3 = 350;
        this.dha4 = 65;
        this.dha5 = 0;
        this.dha6 = 0;

        this.matX2 = new THREE.Matrix4();
        this.matX3 = new THREE.Matrix4();
        this.matX4 = new THREE.Matrix4();
        this.matX5 = new THREE.Matrix4();
        this.matX6 = new THREE.Matrix4();
        this.matX2.set(
            1,0,0,this.dha2,
            0,0,-1,0,
            0,1,0,0,
            0,0,0,1);
        this.matX3.set(
            1,0,0,this.dha3,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1);
        this.matX4.set(
            1,0,0,this.dha4,
            0,0,-1,0,
            0,1,0,0,
            0,0,0,1);
        this.matX5.set(
            1,0,0,this.dha5,
            0,0,1,0,
            0,-1,0,0,
            0,0,0,1);
        this.matX6.set(
            1,0,0,this.dha6,
            0,0,-1,0,
            0,1,0,0,
            0,0,0,1);

        this.matZ0 = new THREE.Matrix4();
        this.matZ0.set(1,0,0,0,
            0,1,0,0,
            0,0,1,this.dhd0,
            0,0,0,1);
        this.matZ1 = new THREE.Matrix4();
        this.matZ2 = new THREE.Matrix4();
        this.matZ3 = new THREE.Matrix4();
        this.matZ4 = new THREE.Matrix4();
        this.matZ5 = new THREE.Matrix4();
        this.matZ6 = new THREE.Matrix4();
    }

    set SetTheta1(value){
        this.matZ1.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd1,
            0,0,0,1
        );
    }
    set SetTheta2(value){
        this.matZ2.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd2,
            0,0,0,1
        );
    }
    set SetTheta3(value){
        this.matZ3.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd3,
            0,0,0,1
        );
    }
    set SetTheta4(value){
        this.matZ4.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd4,
            0,0,0,1
        );
    }
    set SetTheta5(value){
        this.matZ5.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd5,
            0,0,0,1
        );
    }
    set SetTheta6(value){
        this.matZ6.set(
            Math.cos(value), -Math.sin(value), 0, 0,
            Math.sin(value), Math.cos(value), 0, 0,
            0, 0, 1, this.dhd6,
            0,0,0,1
        );
    }

    setNowJointValue(val1, val2, val3, val4, val5, val6){
        this.SetTheta1 = val1;
        this.SetTheta2 = val2;
        this.SetTheta3 = val3;
        this.SetTheta4 = val4;
        this.SetTheta5 = val5;
        this.SetTheta6 = val6;
    }

    MakeInverseTfmMatZ(theta, dhd){
        let temp = new THREE.Matrix4();
        temp.set(
            Math.cos(-theta), -Math.sin(-theta),0,0,
            Math.sin(-theta), Math.cos(theta), 0,0,
            0,0,1,-dhd,
            0,0,0,1
        );
        return temp;
    }

    CalcForwardKinematics(){
        let resMat = new THREE.Matrix4();
        resMat.multiply(this.matZ0);
        resMat.multiply(this.matZ1);
        resMat.multiply(this.matX2);
        resMat.multiply(this.matZ2);
        resMat.multiply(this.matX3);
        resMat.multiply(this.matZ3);
        resMat.multiply(this.matX4);
        resMat.multiply(this.matZ4);
        resMat.multiply(this.matX5);
        resMat.multiply(this.matZ5);
        resMat.multiply(this.matX6);
        resMat.multiply(this.matZ6); 

        return resMat;
    }

    CalcInverseKinematics(aTgtMat){

        let tgtPos = new THREE.Vector3().setFromMatrixPosition(aTgtMat);
        let tgtZDir = new THREE.Vector3().setFromMatrixColumn(aTgtMat, 2);
        const x5 = tgtPos.x - this.dhd6*tgtZDir.x;
        const y5 = tgtPos.y - this.dhd6*tgtZDir.y;
        let z5 = tgtPos.z - this.dhd6*tgtZDir.z;
        z5 -= this.dhd0;
        z5 -= this.dhd1;

        // 逆運動計算
        let arm;
        // コンフィグ値条件分岐
        arm = -this.dha2 + Math.sqrt(x5*x5 + y5*y5 -this.dhd2*this.dhd2);

        // 1軸角度
        const cos1 = ((arm+this.dha2)*x5-this.dhd2*y5)/((arm+this.dha2)*(arm+this.dha2)+this.dhd2*this.dhd2);
        const sin1 = (this.dhd2*x5+(arm+this.dha2)*y5)/((arm+this.dha2)*(arm+this.dha2)+this.dhd2*this.dhd2);
        const theta1 = Math.atan2(sin1,cos1);

        // 3軸角度
        const sinBeta = this.dha4/Math.sqrt(this.dha4*this.dha4+this.dhd4*this.dhd4);
        const cosBeta = this.dhd4/Math.sqrt(this.dha4*this.dha4+this.dhd4*this.dhd4);
        const thetaBeta = Math.atan2(sinBeta,cosBeta);
        const sinGamma = (z5*z5+arm*arm-this.dhd4*this.dhd4-this.dha4*this.dha4-this.dha3*this.dha3)/
        2/Math.sqrt(this.dha3*this.dha3*this.dha4*this.dha4+this.dha3*this.dha3*this.dhd4*this.dhd4);
        // コンフィグ値条件分岐
        const cosGamma = Math.sqrt(1-sinGamma*sinGamma);
        const thetaGamma = Math.atan2(sinGamma,cosGamma);
        const theta3 = thetaGamma-thetaBeta;

        // 2軸角度
        const ee = this.dhd4*Math.cos(theta3)-this.dha4*Math.sin(theta3);
        const ff = this.dhd4*Math.sin(theta3)+this.dha4*Math.cos(theta3);
        const sin2 = (ee*arm+(ff+this.dha3)*z5)/((ff+this.dha3)*(ff+this.dha3)+ee*ee);
        const cos2 = ((ff+this.dha3)*arm-ee*z5)/((ff+this.dha3)*(ff+this.dha3)+ee*ee);
        const theta2 = Math.atan2(sin2,cos2);

        // 姿勢取得
        let temp = new THREE.Matrix4();
        let posMat = this.MakeInverseTfmMatZ(theta1,this.dhd0+this.dhd1).multiply(aTgtMat);
        posMat = temp.getInverse(this.matX2).multiply(aTgtMat);
        posMat = this.MakeInverseTfmMatZ(theta2, this.dhd2).multiply(aTgtMat);
        posMat = temp.getInverse(this.matX3).multiply(aTgtMat);
        posMat = this.MakeInverseTfmMatZ(theta3, this.dhd3).multiply(aTgtMat);
        posMat = temp.getInverse(this.matX4).multiply(aTgtMat);

        const r11 = posMat.elements[0];
        const r12 = posMat.elements[4];
        const r13 = posMat.elements[8];
        const r21 = posMat.elements[1];
        const r22 = posMat.elements[5];
        const r23 = posMat.elements[9];
        const r31 = posMat.elements[2];
        const r32 = posMat.elements[6];
        const r33 = posMat.elements[10];

        // 5軸角度
        let sin5;
        // コンフィグ値条件分岐
        sin5 = Math.sqrt(r13*r13+r23*r23);
        const cos5 = r33;
        const theta5 = Math.atan2(sin5,cos5);

        // 4軸角度
        let sin4,cos4,tan4,theta4;
        if(sin5!=0){
            sin4 = r23/sin5;
            cos4 = r13/sin5;
            theta4 = Math.atan2(sin4,cos4);
        }else{
            tan4 = -(r11-r22)/(r12-r21);
            theta4 = Math.atan(tan4);
        }

        // 6軸角度
        let sin6,cos6,theta6;
        if(sin5!=0){
            sin6 = r32/sin5;
            cos6 = -r31/sin5;
            theta6 = Math.atan2(sin6,cos6);
        }else{
            sin6 = Math.cos(theta4)*r21-Math.sin(theta4)*r11;
            cos6 = Math.sin(theta4)*r11+Math.cos(theta4)*r21;
            theta6 = Math.atan2(sin6,cos6);
        }

        const resJointDeg = [theta1, theta2, theta3, theta4, theta5, theta6];
        //const resJointDeg = [r11,r12,r13,r21,r22,r23,r31,r32,r33];
        return resJointDeg;
    }
}