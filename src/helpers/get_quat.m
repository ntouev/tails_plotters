function [tq, q, qref] = get_quat(ac_data)

    q = double([ac_data.AHRS_REF_QUAT.body_qi ac_data.AHRS_REF_QUAT.body_qx ac_data.AHRS_REF_QUAT.body_qy ac_data.AHRS_REF_QUAT.body_qz]);
    qref = double([ac_data.AHRS_REF_QUAT.ref_qi ac_data.AHRS_REF_QUAT.ref_qx ac_data.AHRS_REF_QUAT.ref_qy ac_data.AHRS_REF_QUAT.ref_qz]);
    [tq,IDXtq,~] = unique(ac_data.AHRS_REF_QUAT.timestamp);
    q = q(IDXtq,:);
    qref = qref(IDXtq,:);

end